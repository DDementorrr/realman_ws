#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "gripper.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using Follow = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ServerGoalHandle<Follow>;

class Ctm2f110Driver final : public rclcpp::Node {
public:
  Ctm2f110Driver() : Node("ctm2f110_driver") {
    port_ = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    baud_ = declare_parameter<int>("baud_rate", 115200);
    dry_run_ = declare_parameter<bool>("dry_run", true);
    readback_ = declare_parameter<bool>("readback", true);
    joint_ = declare_parameter<std::string>("joint_name", "ctm2f110_finger_joint");
    min_position_ = declare_parameter<double>("min_position", 0.0);
    max_position_ = declare_parameter<double>("max_position", 1.0);
    state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    if (!dry_run_ && !sdk_.openSerial(baud_, port_.c_str())) {
      throw std::runtime_error("Unable to open CTM2F110 serial port: " + port_);
    }
    action_ = rclcpp_action::create_server<Follow>(
      this, "follow_joint_trajectory",
      std::bind(&Ctm2f110Driver::goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&Ctm2f110Driver::cancel, this, std::placeholders::_1),
      std::bind(&Ctm2f110Driver::accepted, this, std::placeholders::_1));
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] { publish_state(); });
    RCLCPP_INFO(get_logger(), "CTM2F110 driver ready (%s, dry_run=%s)", port_.c_str(), dry_run_ ? "true" : "false");
  }

  ~Ctm2f110Driver() override {
    cancel_requested_.store(true);
    if (worker_.joinable()) worker_.join();
    if (!dry_run_) sdk_.closeSerial();
  }

private:
  rclcpp_action::GoalResponse goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const Follow::Goal> g) {
    if (g->trajectory.joint_names.size() != 1 || g->trajectory.joint_names[0] != joint_ ||
        g->trajectory.points.empty() || active_.load()) {
      RCLCPP_WARN(get_logger(), "Rejecting trajectory: expected one idle joint named %s", joint_.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    double previous_time = -1.0;
    for (const auto &point : g->trajectory.points) {
      const double stamp = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
      if (point.positions.size() != 1 || !std::isfinite(point.positions[0]) ||
          point.positions[0] < min_position_ || point.positions[0] > max_position_ ||
          stamp < 0.0 || stamp < previous_time) {
        RCLCPP_WARN(get_logger(), "Rejecting invalid CTM2F110 trajectory point");
        return rclcpp_action::GoalResponse::REJECT;
      }
      previous_time = stamp;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  rclcpp_action::CancelResponse cancel(const std::shared_ptr<GoalHandle>) {
    cancel_requested_.store(true);
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  void accepted(const std::shared_ptr<GoalHandle> handle) {
    if (worker_.joinable()) worker_.join();
    cancel_requested_.store(false);
    active_.store(true);
    worker_ = std::thread{[this, handle] { execute(handle); }};
  }

  void execute(const std::shared_ptr<GoalHandle> handle) {
    const auto goal = handle->get_goal();
    auto result = std::make_shared<Follow::Result>();
    result->error_code = Follow::Result::SUCCESSFUL;
    double previous_time = 0.0;
    for (const auto &point : goal->trajectory.points) {
      const double target_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
      const auto wait_time = std::chrono::duration<double>(std::max(0.0, target_time - previous_time));
      const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(wait_time);
      while (std::chrono::steady_clock::now() < deadline) {
        if (cancel_requested_.load() || handle->is_canceling()) {
          handle->canceled(result);
          active_.store(false);
          return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      if (cancel_requested_.load() || handle->is_canceling()) {
        handle->canceled(result);
        active_.store(false);
        return;
      }
      const double target = point.positions[0];
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!dry_run_ && !sdk_.setpos(static_cast<float>(target), static_cast<float>(target))) {
          result->error_code = Follow::Result::PATH_TOLERANCE_VIOLATED;
          handle->abort(result);
          active_.store(false);
          return;
        }
        position_ = target;
      }
      previous_time = target_time;
    }
    handle->succeed(result);
    active_.store(false);
  }

  void publish_state() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!dry_run_ && readback_) {
      float actuator1 = 0.0F;
      float actuator2 = 0.0F;
      if (sdk_.readpos(actuator1, actuator2)) {
        position_ = std::clamp(0.5 * (static_cast<double>(actuator1) + actuator2),
          min_position_, max_position_);
        readback_warned_ = false;
      } else if (!readback_warned_) {
        RCLCPP_WARN(get_logger(), "CTM2F110 present-position readback failed; retaining last state");
        readback_warned_ = true;
      }
    }
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name = {joint_, "ctm2f110_rightout_joint", "ctm2f110_leftinn_joint", "ctm2f110_rightinn_joint", "ctm2f110_left_kckle_joint", "ctm2f110_right_kckle_joint"};
    msg.position = {position_, position_, -position_, -position_, position_, position_};
    state_pub_->publish(msg);
  }

  std::string port_, joint_;
  int baud_;
  bool dry_run_, readback_;
  bool readback_warned_{false};
  double min_position_, max_position_, position_{0.3};
  std::atomic_bool active_{false};
  std::atomic_bool cancel_requested_{false};
  GripperSDK sdk_; std::mutex mutex_;
  std::thread worker_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Server<Follow>::SharedPtr action_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<Ctm2f110Driver>()); }
  catch (const std::exception &e) { fprintf(stderr, "%s\n", e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown();
  return 0;
}
