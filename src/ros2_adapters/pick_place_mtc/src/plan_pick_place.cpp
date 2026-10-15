#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <map>
#include <memory>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/task.h>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <moveit_task_constructor_msgs/msg/task_description.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace mtc = moveit::task_constructor;
using namespace std::chrono_literals;

class PickPlacePlanner
{
public:
  PickPlacePlanner()
    : node_(std::make_shared<rclcpp::Node>(
          "plan_pick_place", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)))
  {
    object_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/object_pose", 10, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(poses_mutex_);
        object_pose_ = *msg;
      });
    place_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/place_pose", 10, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(poses_mutex_);
        place_pose_ = *msg;
      });
    scene_client_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    plan_ready_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      "/pick_place/plan_ready", rclcpp::QoS(1).reliable().transient_local());
    solution_pub_ = node_->create_publisher<moveit_task_constructor_msgs::msg::Solution>(
      "/pick_place/solution", rclcpp::QoS(1).reliable().transient_local());
    description_pub_ = node_->create_publisher<moveit_task_constructor_msgs::msg::TaskDescription>(
      "/pick_place/task_description", rclcpp::QoS(1).reliable().transient_local());
    metrics_pub_ = node_->create_publisher<std_msgs::msg::String>(
      "/pick_place/planning_metrics", rclcpp::QoS(1).reliable().transient_local());
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base() const
  {
    return node_->get_node_base_interface();
  }

  bool run()
  {
    geometry_msgs::msg::PoseStamped place_pose;
    bool scene_ready = false;
    // RViz and Gazebo GUI startup can take over a minute on a desktop GPU.
    // Keep the planner alive while MoveIt and the PlanningScene become ready.
    for (int attempt = 0; rclcpp::ok() && attempt < 180; ++attempt) {
      bool poses_ready = false;
      {
        std::lock_guard<std::mutex> lock(poses_mutex_);
        if (object_pose_ && place_pose_) {
          place_pose = *place_pose_;
          poses_ready = true;
        }
      }
      scene_ready = poses_ready && has_known_scene();
      if (scene_ready) {
        break;
      }
      std::this_thread::sleep_for(500ms);
    }
    if (!scene_ready) {
      RCLCPP_ERROR(node_->get_logger(), "Timed out waiting for known poses and MoveIt collision scene");
      return false;
    }

    task_ = make_task(place_pose);
    try {
      task_.init();
    } catch (const mtc::InitStageException& error) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "MTC initialization failed: " << error);
      return false;
    }
    const int candidates = node_->get_parameter_or("planning_candidates", 5);
    const auto weights = node_->get_parameter_or(
      "joint_motion_weights", std::vector<double>{5.0, 4.0, 3.0, 2.0, 1.0, 1.0});
    if (candidates < 1 || candidates > 20 || weights.size() != 6 ||
        std::any_of(weights.begin(), weights.end(), [](double w) { return !std::isfinite(w) || w <= 0; })) {
      RCLCPP_ERROR(node_->get_logger(), "Invalid planning_candidates or joint_motion_weights");
      return false;
    }
    if (!task_.plan(candidates)) {
      RCLCPP_ERROR(node_->get_logger(), "MTC could not plan the complete Pick and Place task");
      task_.printState();
      return false;
    }
    // Score actual waypoint travel, including stage boundaries. Bounded
    // revolute joints use raw deltas: wrapping would hide a commanded long turn.
    mtc::SolutionBaseConstPtr selected;
    double best = std::numeric_limits<double>::infinity();
    std::string report = "weighted_joint_travel_rad; weights=";
    for (double w : weights) report += std::to_string(w) + ",";
    size_t index = 0, selected_index = 0;
    for (const auto& candidate : task_.solutions()) {
      moveit_task_constructor_msgs::msg::Solution msg;
      candidate->toMsg(msg, &task_.introspection());
      std::map<std::string, double> previous;
      double score = 0.0;
      for (const auto& part : msg.sub_trajectory) {
        const auto& trajectory = part.trajectory.joint_trajectory;
        for (const auto& point : trajectory.points) {
          for (size_t j = 0; j < trajectory.joint_names.size(); ++j) {
            const auto& name = trajectory.joint_names[j];
            for (size_t axis = 0; axis < weights.size(); ++axis) {
              if (name != "joint" + std::to_string(axis + 1)) continue;
              if (j >= point.positions.size() || !std::isfinite(point.positions[j])) {
                score = std::numeric_limits<double>::infinity();
                continue;
              }
              if (previous.count(name)) score += weights[axis] * std::abs(point.positions[j] - previous[name]);
              previous[name] = point.positions[j];
            }
          }
        }
      }
      if (previous.size() != 6) score = std::numeric_limits<double>::infinity();
      report += " candidate[" + std::to_string(index) + "]=" + std::to_string(score);
      if (score < best) { best = score; selected = candidate; selected_index = index; }
      ++index;
    }
    if (!selected) {
      RCLCPP_ERROR(node_->get_logger(), "No complete candidate with finite six-axis trajectory score");
      return false;
    }
    report += " selected=" + std::to_string(selected_index);
    std_msgs::msg::String metrics;
    metrics.data = report;
    metrics_pub_->publish(metrics);
    RCLCPP_INFO(node_->get_logger(), "%s", report.c_str());
    task_.introspection().publishSolution(*selected);
    moveit_task_constructor_msgs::msg::TaskDescription description;
    task_.introspection().fillTaskDescription(description);
    moveit_task_constructor_msgs::msg::Solution solution;
    selected->toMsg(solution, &task_.introspection());
    description_pub_->publish(description);
    solution_pub_->publish(solution);
    std_msgs::msg::Bool ready;
    ready.data = true;
    plan_ready_pub_->publish(ready);
    RCLCPP_INFO(node_->get_logger(), "Complete MTC solution published; execution is a later stage gate");
    return true;
  }

private:
  bool has_known_scene()
  {
    if (!scene_client_->service_is_ready()) {
      return false;
    }
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components = moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;
    auto future = scene_client_->async_send_request(request);
    if (future.wait_for(500ms) != std::future_status::ready) {
      return false;
    }
    bool object_found = false;
    bool table_found = false;
    for (const auto& object : future.get()->scene.world.collision_objects) {
      object_found |= object.id == "pick_object";
      table_found |= object.id == "pick_table";
    }
    return object_found && table_found;
  }

  mtc::Task make_task(const geometry_msgs::msg::PoseStamped& place_pose)
  {
    constexpr const char* arm = "rm65_arm";
    constexpr const char* hand = "rm65_hand";
    constexpr const char* tool = "ctm2f110_tool0";
    constexpr const char* object = "pick_object";
    constexpr const char* table = "pick_table";
    const double grasp_spin = node_->get_parameter_or("grasp_spin", 0.0);
    const double grasp_depth = node_->get_parameter_or("grasp_depth", 0.0);
    const double grasp_lateral_offset = node_->get_parameter_or("grasp_lateral_offset", 0.0);
    const double grip_target = node_->get_parameter_or("grip_target", 0.6);
    const double open_target = node_->get_parameter_or("open_target", 0.8);
    const double gripper_motion_scaling = node_->get_parameter_or("gripper_motion_scaling", 1.0);
    const double motion_velocity_scaling = node_->get_parameter_or("motion_velocity_scaling", 1.0);
    const double motion_acceleration_scaling = node_->get_parameter_or("motion_acceleration_scaling", 1.0);
    const double transport_velocity_scaling = node_->get_parameter_or("transport_velocity_scaling", 1.0);
    const double transport_acceleration_scaling = node_->get_parameter_or("transport_acceleration_scaling", 1.0);
    const bool transport_keep_orientation = node_->get_parameter_or("transport_keep_orientation", false);
    const double transport_orientation_tolerance = node_->get_parameter_or("transport_orientation_tolerance", 0.2);
    const double place_lower_min = node_->get_parameter_or("place_lower_min", 0.08);
    const double place_lower_max = node_->get_parameter_or("place_lower_max", 0.12);
    const double place_height_offset = node_->get_parameter_or("place_height_offset", 0.0);
    if (!std::isfinite(grasp_spin) || !std::isfinite(grasp_depth) ||
        !std::isfinite(grasp_lateral_offset) || !std::isfinite(grip_target) ||
        !std::isfinite(open_target) ||
        !std::isfinite(gripper_motion_scaling) || gripper_motion_scaling <= 0.0 ||
        gripper_motion_scaling > 1.0 ||
        grasp_depth < 0.0 || grasp_depth > 0.05 ||
        std::abs(grasp_lateral_offset) > 0.02 ||
        grip_target < 0.05 || grip_target > 0.8 ||
        open_target < 0.8 || open_target > 1.0 ||
        !std::isfinite(motion_velocity_scaling) || !std::isfinite(motion_acceleration_scaling) ||
        motion_velocity_scaling <= 0.0 || motion_velocity_scaling > 1.0 ||
        motion_acceleration_scaling <= 0.0 || motion_acceleration_scaling > 1.0 ||
        !std::isfinite(transport_velocity_scaling) || !std::isfinite(transport_acceleration_scaling) ||
        transport_velocity_scaling <= 0.0 || transport_velocity_scaling > 1.0 ||
        transport_acceleration_scaling <= 0.0 || transport_acceleration_scaling > 1.0 ||
        !std::isfinite(transport_orientation_tolerance) ||
        transport_orientation_tolerance < 0.01 || transport_orientation_tolerance > 0.6 ||
        !std::isfinite(place_lower_min) || !std::isfinite(place_lower_max) ||
        place_lower_min <= 0.0 || place_lower_max > 0.15 ||
        place_lower_min > place_lower_max ||
        !std::isfinite(place_height_offset) ||
        place_height_offset < 0.0 || place_height_offset > 0.05) {
      throw std::invalid_argument("Pick and Place motion parameter is outside its supported range");
    }

    mtc::Task task;
    task.stages()->setName("RM65B known-pose Pick and Place");
    task.loadRobotModel(node_);
    task.setProperty("group", std::string(arm));
    task.setProperty("eef", std::string("ctm2f110"));
    task.setProperty("ik_frame", std::string(tool));
    std::vector<std::string> gripper_links;
    for (const auto& link : task.getRobotModel()->getLinkModelNamesWithCollisionGeometry()) {
      if (link.rfind("ctm2f110_", 0) == 0) {
        gripper_links.push_back(link);
      }
    }
    if (gripper_links.empty()) {
      throw std::runtime_error("No CTM2F110 collision links in robot model");
    }

    auto ompl = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");
    ompl->setPlannerId("RRTConnectkConfigDefault");
    ompl->setMaxVelocityScalingFactor(motion_velocity_scaling);
    ompl->setMaxAccelerationScalingFactor(motion_acceleration_scaling);
    auto transport_ompl = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "ompl");
    transport_ompl->setPlannerId("RRTConnectkConfigDefault");
    transport_ompl->setMaxVelocityScalingFactor(transport_velocity_scaling);
    transport_ompl->setMaxAccelerationScalingFactor(transport_acceleration_scaling);
    auto joints = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    joints->setMaxVelocityScalingFactor(gripper_motion_scaling);
    joints->setMaxAccelerationScalingFactor(gripper_motion_scaling);
    auto cartesian = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian->setStepSize(0.005);
    cartesian->setMaxVelocityScalingFactor(std::min(0.10, motion_velocity_scaling));
    cartesian->setMaxAccelerationScalingFactor(std::min(0.10, motion_acceleration_scaling));

    task.add(std::make_unique<mtc::stages::CurrentState>("current state"));
    {
      auto home = std::make_unique<mtc::stages::MoveTo>("Home", ompl);
      home->setGroup(arm);
      home->setGoal("ready");
      task.add(std::move(home));
    }
    {
      auto open = std::make_unique<mtc::stages::MoveTo>("open gripper", joints);
      open->setGroup(hand);
      if (open_target == 0.8) {
        open->setGoal("open");
      } else {
        open->setGoal(std::map<std::string, double>{{"ctm2f110_finger_joint", open_target}});
      }
      task.add(std::move(open));
    }
    auto contact = std::make_unique<mtc::stages::ModifyPlanningScene>("allow gripper contact");
    contact->allowCollisions(object, gripper_links, true);
    mtc::Stage* contact_ptr = contact.get();
    task.add(std::move(contact));
    {
      auto connect = std::make_unique<mtc::stages::Connect>(
        "Pre-Grasp (OMPL)", mtc::stages::Connect::GroupPlannerVector{{arm, ompl}});
      connect->setTimeout(10.0);
      task.add(std::move(connect));
    }

    mtc::Stage* attached = nullptr;
    {
      auto pick = std::make_unique<mtc::SerialContainer>("pick");
      task.properties().exposeTo(pick->properties(), {"eef", "group", "ik_frame"});
      pick->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});
      auto approach = std::make_unique<mtc::stages::MoveRelative>("Approach (Cartesian)", cartesian);
      approach->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      approach->setIKFrame(tool);
      approach->setMinMaxDistance(0.08, 0.12);
      geometry_msgs::msg::Vector3Stamped down;
      down.header.frame_id = "world";
      down.vector.x = 1.0;
      approach->setDirection(down);
      pick->insert(std::move(approach));

      auto grasp = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp poses");
      grasp->properties().configureInitFrom(mtc::Stage::PARENT);
      grasp->setObject(object);
      if (open_target == 0.8) {
        grasp->setPreGraspPose("open");
      } else {
        moveit_msgs::msg::RobotState pregrasp;
        pregrasp.is_diff = true;
        pregrasp.joint_state.name = {"ctm2f110_finger_joint"};
        pregrasp.joint_state.position = {open_target};
        grasp->setPreGraspPose(pregrasp);
      }
      // The known upright box uses a verified side grasp; add yaw sampling once
      // full-task planning is stable so this gate stays bounded and repeatable.
      grasp->setAngleDelta(2 * M_PI);
      grasp->setMonitoredStage(contact_ptr);
      auto ik = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(grasp));
      ik->setTimeout(0.25);
      ik->setMaxIKSolutions(2);
      ik->setMinSolutionDistance(0.5);
      Eigen::Isometry3d grasp_frame = Eigen::Isometry3d::Identity();
      // The default keeps Stage 3's upright fixed-joint anchor. A 90 degree
      // tool-axis spin makes the physical jaws close across the box width.
      grasp_frame.linear() =
        (Eigen::AngleAxisd(-grasp_spin, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY())).toRotationMatrix();
      // The CTM2F110 pad centers extend about 30 mm beyond tool0. In contact
      // mode, offset the target along tool Z so the pads surround the box
      // center instead of touching only its far edge.
      grasp_frame.translation().z() = grasp_depth;
      grasp_frame.translation().x() = grasp_lateral_offset;
      ik->setIKFrame(grasp_frame, tool);
      ik->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      pick->insert(std::move(ik));

      auto support = std::make_unique<mtc::stages::ModifyPlanningScene>("allow table support contact");
      support->allowCollisions(object, table, true);
      pick->insert(std::move(support));
      auto close = std::make_unique<mtc::stages::MoveTo>("Close Gripper", joints);
      close->setGroup(hand);
      close->setGoal(std::map<std::string, double>{{"ctm2f110_finger_joint", grip_target}});
      pick->insert(std::move(close));
      auto attach = std::make_unique<mtc::stages::ModifyPlanningScene>("Attach in PlanningScene");
      attach->attachObject(object, tool);
      attached = attach.get();
      pick->insert(std::move(attach));
      auto lift = std::make_unique<mtc::stages::MoveRelative>("Lift (Cartesian)", cartesian);
      lift->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      lift->setIKFrame(tool);
      lift->setMinMaxDistance(0.03, 0.05);
      geometry_msgs::msg::Vector3Stamped up;
      up.header.frame_id = "world";
      up.vector.z = 1.0;
      lift->setDirection(up);
      pick->insert(std::move(lift));
      task.add(std::move(pick));
    }
    {
      auto connect = std::make_unique<mtc::stages::Connect>(
        "Transport (OMPL)", mtc::stages::Connect::GroupPlannerVector{{arm, transport_ompl}});
      connect->setTimeout(transport_keep_orientation ? 30.0 : 10.0);
      if (transport_keep_orientation) {
        // The attached-object collision model alone does not keep a friction
        // grasp upright. Bound all rotation-vector axes relative to release
        // so OMPL cannot insert a redundant wrist turn during translation.
        const auto& q = place_pose.pose.orientation;
        Eigen::Quaterniond object_orientation(q.w, q.x, q.y, q.z);
        if (!object_orientation.coeffs().allFinite() || object_orientation.norm() < 1e-6) {
          throw std::invalid_argument("Invalid place orientation for transport constraint");
        }
        const Eigen::Quaterniond tool_orientation = object_orientation.normalized() *
          Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(grasp_spin, Eigen::Vector3d::UnitZ()));
        moveit_msgs::msg::OrientationConstraint orientation;
        orientation.header.frame_id = place_pose.header.frame_id;
        orientation.link_name = tool;
        orientation.orientation.x = tool_orientation.x();
        orientation.orientation.y = tool_orientation.y();
        orientation.orientation.z = tool_orientation.z();
        orientation.orientation.w = tool_orientation.w();
        orientation.absolute_x_axis_tolerance = transport_orientation_tolerance;
        orientation.absolute_y_axis_tolerance = transport_orientation_tolerance;
        orientation.absolute_z_axis_tolerance = transport_orientation_tolerance;
        orientation.parameterization = moveit_msgs::msg::OrientationConstraint::ROTATION_VECTOR;
        orientation.weight = 1.0;
        moveit_msgs::msg::Constraints constraints;
        constraints.orientation_constraints.push_back(orientation);
        connect->setPathConstraints(constraints);
      }
      task.add(std::move(connect));
    }
    {
      auto place = std::make_unique<mtc::SerialContainer>("place");
      task.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
      place->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});
      auto lower = std::make_unique<mtc::stages::MoveRelative>("Place (Cartesian)", cartesian);
      lower->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      lower->setIKFrame(tool);
      lower->setMinMaxDistance(place_lower_min, place_lower_max);
      geometry_msgs::msg::Vector3Stamped down;
      down.header.frame_id = "world";
      down.vector.z = -1.0;
      lower->setDirection(down);
      place->insert(std::move(lower));

      // Preserve the requested object orientation instead of sampling box
      // symmetries that can add a half-turn during carry. ComputeIK retains
      // the grasp transform through its attached-object IK frame below.
      auto generate = std::make_unique<mtc::stages::GeneratePose>("generate place pose");
      generate->properties().configureInitFrom(mtc::Stage::PARENT);
      auto release_pose = place_pose;
      release_pose.pose.position.z += place_height_offset;
      generate->setPose(release_pose);
      generate->setMonitoredStage(attached);
      auto ik = std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(generate));
      ik->setTimeout(1.0);
      ik->setMaxIKSolutions(8);
      ik->setIKFrame(object);
      ik->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      ik->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      place->insert(std::move(ik));

      auto open = std::make_unique<mtc::stages::MoveTo>("Open Gripper", joints);
      open->setGroup(hand);
      if (open_target == 0.8) {
        open->setGoal("open");
      } else {
        open->setGoal(std::map<std::string, double>{{"ctm2f110_finger_joint", open_target}});
      }
      place->insert(std::move(open));
      auto detach = std::make_unique<mtc::stages::ModifyPlanningScene>("Detach in PlanningScene");
      detach->detachObject(object, tool);
      place->insert(std::move(detach));
      auto retreat = std::make_unique<mtc::stages::MoveRelative>("Retreat (Cartesian)", cartesian);
      retreat->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      retreat->setIKFrame(tool);
      retreat->setMinMaxDistance(0.10, 0.14);
      geometry_msgs::msg::Vector3Stamped up;
      up.header.frame_id = "world";
      up.vector.x = -1.0;
      retreat->setDirection(up);
      place->insert(std::move(retreat));
      auto no_contact = std::make_unique<mtc::stages::ModifyPlanningScene>("restore collision rules");
      no_contact->allowCollisions(object, gripper_links, false);
      place->insert(std::move(no_contact));
      task.add(std::move(place));
    }
    {
      auto home = std::make_unique<mtc::stages::MoveTo>("return Home", ompl);
      home->setGroup(arm);
      home->setGoal("ready");
      task.add(std::move(home));
    }
    return task;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr object_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr place_sub_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr scene_client_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr plan_ready_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metrics_pub_;
  rclcpp::Publisher<moveit_task_constructor_msgs::msg::Solution>::SharedPtr solution_pub_;
  rclcpp::Publisher<moveit_task_constructor_msgs::msg::TaskDescription>::SharedPtr description_pub_;
  std::mutex poses_mutex_;
  std::optional<geometry_msgs::msg::PoseStamped> object_pose_;
  std::optional<geometry_msgs::msg::PoseStamped> place_pose_;
  mtc::Task task_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto planner = std::make_shared<PickPlacePlanner>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(planner->node_base());
  std::thread spin_thread([&executor] { executor.spin(); });
  const bool success = planner->run();
  if (success) {
    // Keep MTC introspection alive for RViz until interrupted.
    spin_thread.join();
  } else {
    executor.cancel();
    spin_thread.join();
  }
  rclcpp::shutdown();
  return success ? 0 : 1;
}
