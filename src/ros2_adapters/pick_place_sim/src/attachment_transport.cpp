#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/contacts.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/entity_plugin_v.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

class AttachmentTransport : public rclcpp::Node
{
public:
  AttachmentTransport() : rclcpp::Node("pick_place_attachment_transport")
  {
    state_pub_ = create_publisher<std_msgs::msg::String>(
      "/pick_place/sim/attachment_state", rclcpp::QoS(1).reliable().transient_local());
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/pick_place/sim/object_pose", 10);
    anchor_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/pick_place/sim/anchor_pose", 10);
    tool_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/pick_place/sim/tool_pose", 10);
    left_inner_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/pick_place/sim/left_inner_pose", 10);
    right_inner_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/pick_place/sim/right_inner_pose", 10);
    contact_pub_ = create_publisher<std_msgs::msg::String>(
      "/pick_place/sim/contact_state", 10);
    pick_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/object_pose", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        pick_pose_ = *message;
        have_pick_pose_ = true;
      });
    attach_pub_ = gz_node_.Advertise<gz::msgs::Empty>("/pick_place/gz_attach");
    detach_pub_ = gz_node_.Advertise<gz::msgs::Empty>("/pick_place/gz_detach");
    if (!gz_node_.Subscribe("/pick_place/gz_attachment_state", &AttachmentTransport::on_state, this) ||
        !gz_node_.Subscribe("/world/empty/pose/info", &AttachmentTransport::on_poses, this) ||
        !gz_node_.Subscribe("/world/empty/dynamic_pose/info", &AttachmentTransport::on_poses, this)) {
      throw std::runtime_error("Could not subscribe to Gazebo attachment state or world poses");
    }
    gz_node_.Subscribe(
      "/world/empty/model/pick_object/link/body/sensor/grasp_contacts/contact",
      &AttachmentTransport::on_contacts, this);
    set_service_ = create_service<std_srvs::srv::SetBool>(
      "/pick_place/sim/set_attachment",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr request,
             std_srvs::srv::SetBool::Response::SharedPtr response) {
        set_attachment(request->data, *response);
      });
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/pick_place/sim/reset_object",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             const std_srvs::srv::Trigger::Response::SharedPtr response) {
        geometry_msgs::msg::Pose pick_pose;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (state_ != "detached") {
            response->message = "Cannot reset a Gazebo object while attached";
            return;
          }
          if (!have_pick_pose_ || pick_pose_.header.frame_id != "world") {
            response->message = "No world-frame known pick pose available";
            return;
          }
          pick_pose = pick_pose_.pose;
        }
        gz::msgs::Pose request;
        request.set_name("pick_object");
        request.mutable_position()->set_x(pick_pose.position.x);
        request.mutable_position()->set_y(pick_pose.position.y);
        request.mutable_position()->set_z(pick_pose.position.z);
        request.mutable_orientation()->set_x(pick_pose.orientation.x);
        request.mutable_orientation()->set_y(pick_pose.orientation.y);
        request.mutable_orientation()->set_z(pick_pose.orientation.z);
        request.mutable_orientation()->set_w(pick_pose.orientation.w);
        gz::msgs::Boolean reply;
        bool result = false;
        response->success = gz_node_.Request(
          "/world/empty/set_pose", request, 5000, reply, result) && result && reply.data();
        response->message = response->success ? "Gazebo object reset to pick pose" :
                                            "Gazebo rejected object pose reset";
      });
    // No joint exists yet. Loading it only after the robot reaches the object
    // avoids the plugin's built-in initial attachment at the robot's home pose.
    std_msgs::msg::String initial_state;
    initial_state.data = "detached";
    state_pub_->publish(initial_state);
  }

private:
  void on_contacts(const gz::msgs::Contacts& message)
  {
    const auto stamp = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(contact_publish_mutex_);
      if (stamp - last_contact_publish_ < 20ms) {
        return;
      }
      last_contact_publish_ = stamp;
    }
    std::set<std::string> partners;
    for (const auto& contact : message.contact()) {
      const auto& first = contact.collision1().name();
      const auto& second = contact.collision2().name();
      if (first.find("pick_object::body::collision") != std::string::npos) {
        partners.insert(second);
      } else if (second.find("pick_object::body::collision") != std::string::npos) {
        partners.insert(first);
      }
    }
    std_msgs::msg::String ros_message;
    for (const auto& partner : partners) {
      if (!ros_message.data.empty()) {
        ros_message.data += ";";
      }
      ros_message.data += partner;
    }
    contact_pub_->publish(ros_message);
  }

  void on_state(const gz::msgs::StringMsg& message)
  {
    const std::string value = message.data();
    if (value != "attached" && value != "detached") {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      state_ = value;
    }
    cv_.notify_all();
    std_msgs::msg::String ros_message;
    ros_message.data = value;
    state_pub_->publish(ros_message);
    RCLCPP_INFO(get_logger(), "Gazebo attachment state: %s", value.c_str());
  }

  void on_poses(const gz::msgs::Pose_V& message)
  {
    for (const auto& pose : message.pose()) {
      if (pose.name() == "rm65b") {
        robot_entity_.store(pose.id());
      }
      const bool object = pose.name() == "pick_object";
      const bool anchor = pose.name() == "pick_place_grasp_anchor";
      const bool tool = pose.name() == "Link6";
      // Fixed-joint reduction folds each pad into the inner finger link.
      // Publish link world poses; the diagnostic applies the URDF pad offset.
      const bool left_inner = pose.name() == "ctm2f110_leftinn_Link";
      const bool right_inner = pose.name() == "ctm2f110_rightinn_Link";
      if (!object && !anchor && !tool && !left_inner && !right_inner) {
        continue;
      }
      geometry_msgs::msg::PoseStamped ros_message;
      ros_message.header.frame_id = "world";
      ros_message.header.stamp = now();
      ros_message.pose.position.x = pose.position().x();
      ros_message.pose.position.y = pose.position().y();
      ros_message.pose.position.z = pose.position().z();
      ros_message.pose.orientation.x = pose.orientation().x();
      ros_message.pose.orientation.y = pose.orientation().y();
      ros_message.pose.orientation.z = pose.orientation().z();
      ros_message.pose.orientation.w = pose.orientation().w();
      if (object) {
        pose_pub_->publish(ros_message);
      } else if (anchor) {
        anchor_pose_pub_->publish(ros_message);
      } else if (tool) {
        tool_pose_pub_->publish(ros_message);
      } else if (left_inner) {
        left_inner_pose_pub_->publish(ros_message);
      } else {
        right_inner_pose_pub_->publish(ros_message);
      }
    }
  }

  void set_attachment(bool attach, std_srvs::srv::SetBool::Response& response)
  {
    const std::string desired = attach ? "attached" : "detached";
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (state_ == desired) {
        response.success = true;
        response.message = "Gazebo already " + desired;
        return;
      }
    }
    if (attach && !plugin_loaded_) {
      if (robot_entity_.load() == 0) {
        response.message = "Gazebo robot model entity is not yet available";
        return;
      }
      gz::msgs::EntityPlugin_V request;
      request.mutable_entity()->set_id(robot_entity_.load());
      request.mutable_entity()->set_name("rm65b");
      request.mutable_entity()->set_type(gz::msgs::Entity::MODEL);
      auto* plugin = request.add_plugins();
      plugin->set_name("gz::sim::systems::DetachableJoint");
      plugin->set_filename("gz-sim-detachable-joint-system");
      plugin->set_innerxml(
        "<parent_link>pick_place_grasp_anchor</parent_link>"
        "<child_model>pick_object</child_model>"
        "<child_link>body</child_link>"
        "<attach_topic>/pick_place/gz_attach</attach_topic>"
        "<detach_topic>/pick_place/gz_detach</detach_topic>"
        "<output_topic>/pick_place/gz_attachment_state</output_topic>");
      gz::msgs::Boolean reply;
      bool result = false;
      if (!gz_node_.Request("/world/empty/entity/system/add", request, 5000, reply, result) ||
          !result || !reply.data()) {
        response.message = "Gazebo failed to load DetachableJoint at grasp time";
        return;
      }
      plugin_loaded_ = true;
      RCLCPP_INFO(get_logger(), "Loaded Gazebo DetachableJoint at grasp time");
    }
    for (int attempt = 0; attempt < 8; ++attempt) {
      gz::msgs::Empty message;
      if (attach) {
        attach_pub_.Publish(message);
      } else {
        detach_pub_.Publish(message);
      }
      std::unique_lock<std::mutex> lock(mutex_);
      if (cv_.wait_for(lock, 500ms, [this, &desired] { return state_ == desired; })) {
        response.success = true;
        response.message = "Gazebo confirmed " + desired;
        return;
      }
    }
    response.message = "Gazebo did not confirm " + desired + " within 4 s";
  }

  gz::transport::Node gz_node_;
  gz::transport::Node::Publisher attach_pub_;
  gz::transport::Node::Publisher detach_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr anchor_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tool_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_inner_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_inner_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr contact_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pick_pose_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  std::mutex mutex_;
  std::mutex contact_publish_mutex_;
  std::chrono::steady_clock::time_point last_contact_publish_{};
  std::condition_variable cv_;
  std::string state_ = "detached";
  bool plugin_loaded_ = false;
  std::atomic<uint64_t> robot_entity_{0};
  geometry_msgs::msg::PoseStamped pick_pose_;
  bool have_pick_pose_ = false;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<AttachmentTransport>());
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("attachment_transport"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
