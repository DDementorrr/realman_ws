#include <atomic>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {
bool validQuaternion(const geometry_msgs::msg::Quaternion &q) {
  const auto norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  return std::isfinite(norm) && norm > 1e-6;
}
} // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("target_pose_executor");

  const auto planning_group =
      node->declare_parameter<std::string>("planning_group", "rm65_arm");
  const auto target_topic = node->declare_parameter<std::string>(
      "target_topic", "/manipulation/rm65b/target_pose");
  const auto planner_id =
      node->declare_parameter<std::string>("planner_id", "");
  const auto planning_time =
      node->declare_parameter<double>("planning_time", 5.0);
  const auto velocity =
      node->declare_parameter<double>("velocity_scaling", 0.2);
  const auto acceleration =
      node->declare_parameter<double>("acceleration_scaling", 0.2);
  const auto execute = node->declare_parameter<bool>("execute", true);

  moveit::planning_interface::MoveGroupInterface move_group(node,
                                                            planning_group);
  move_group.setPlanningTime(planning_time);
  move_group.setMaxVelocityScalingFactor(velocity);
  move_group.setMaxAccelerationScalingFactor(acceleration);
  if (!planner_id.empty()) {
    move_group.setPlannerId(planner_id);
  }

  std::atomic_bool busy{false};
  auto target_callback_group =
      node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = target_callback_group;
  auto subscription =
      node->create_subscription<geometry_msgs::msg::PoseStamped>(
          target_topic, rclcpp::QoS(10),
          [&](const geometry_msgs::msg::PoseStamped::SharedPtr target) {
            if (busy.exchange(true)) {
              RCLCPP_WARN(node->get_logger(),
                          "Ignoring target while a plan is executing");
              return;
            }
            if (!validQuaternion(target->pose.orientation)) {
              RCLCPP_ERROR(node->get_logger(),
                           "Rejected target with invalid quaternion");
              busy = false;
              return;
            }

            auto goal = *target;
            if (goal.header.frame_id.empty()) {
              goal.header.frame_id = move_group.getPlanningFrame();
            }
            move_group.setStartStateToCurrentState();
            if (!move_group.setPoseTarget(goal)) {
              RCLCPP_ERROR(node->get_logger(),
                           "Target pose is invalid for group '%s'",
                           planning_group.c_str());
              busy = false;
              return;
            }

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            const bool planned = static_cast<bool>(move_group.plan(plan));
            if (!planned) {
              RCLCPP_ERROR(node->get_logger(), "Planning failed");
            } else if (execute) {
              const auto result = move_group.execute(plan);
              if (result == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(node->get_logger(), "Target reached successfully");
              } else {
                RCLCPP_ERROR(node->get_logger(),
                             "Execution failed with MoveIt code %d",
                             result.val);
              }
            } else {
              RCLCPP_INFO(node->get_logger(),
                          "Plan succeeded; execution disabled by parameter");
            }
            move_group.clearPoseTargets();
            busy = false;
          },
          subscription_options);

  RCLCPP_INFO(node->get_logger(), "Listening on %s for planning group %s",
              target_topic.c_str(), planning_group.c_str());
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
