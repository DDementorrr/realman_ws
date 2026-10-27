#pragma once

#include <array>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <moveit/robot_model/robot_model.h>
#include <rclcpp/rclcpp.hpp>

namespace pick_place_mtc
{
struct TaskConfig
{
  std::string arm_group, hand_group, end_effector, tool_link, hand_joint;
  std::string object_id, support_id, motion_frame, home_state, pipeline_id, planner_id, task_name;
  std::vector<std::string> contact_links, scoring_joints;
  std::vector<double> joint_motion_weights;
  std::array<double, 3> approach_direction, lift_direction, lower_direction, retreat_direction;
  double approach_min, approach_max, lift_min, lift_max, retreat_min, retreat_max;
  double grasp_pitch, grasp_spin, grasp_depth, grasp_lateral_offset, grip_target, open_target;
  double gripper_motion_scaling, motion_velocity_scaling, motion_acceleration_scaling;
  double transport_velocity_scaling, transport_acceleration_scaling, transport_orientation_tolerance;
  double place_lower_min, place_lower_max, place_height_offset, input_wait_timeout_sec;
  double cartesian_step_m, cartesian_scaling_cap, connect_timeout_sec, constrained_connect_timeout_sec;
  bool transport_keep_orientation;
  int planning_candidates;

  static void require(bool condition, const std::string& parameter, const std::string& reason)
  {
    if (!condition) throw std::invalid_argument(parameter + ": " + reason);
  }

  template <typename T>
  static T read(const rclcpp::Node& node, const std::string& name)
  {
    T value{};
    try {
      require(node.get_parameter(name, value), name, "required parameter is missing");
    } catch (const std::exception& error) {
      throw std::invalid_argument(name + ": " + error.what());
    }
    return value;
  }

  static TaskConfig load(const rclcpp::Node& node)
  {
    TaskConfig c;
#define READ_STRING(name) c.name = read<std::string>(node, #name); require(!c.name.empty(), #name, "must be nonempty")
    READ_STRING(arm_group); READ_STRING(hand_group); READ_STRING(end_effector);
    READ_STRING(tool_link); READ_STRING(hand_joint); READ_STRING(object_id);
    READ_STRING(support_id); READ_STRING(motion_frame); READ_STRING(home_state);
    READ_STRING(pipeline_id); READ_STRING(planner_id); READ_STRING(task_name);
#undef READ_STRING
    require(c.object_id != c.support_id, "support_id", "must differ from object_id");
    auto names = [&node](const std::string& name) {
      auto values = read<std::vector<std::string>>(node, name);
      std::set<std::string> unique(values.begin(), values.end());
      require(!values.empty() && !unique.count("") && unique.size() == values.size(), name,
              "must contain unique nonempty names");
      return values;
    };
    c.contact_links = names("contact_links");
    c.scoring_joints = names("scoring_joints");
    c.joint_motion_weights = read<std::vector<double>>(node, "joint_motion_weights");
    require(c.joint_motion_weights.size() == c.scoring_joints.size(), "joint_motion_weights", "length must match scoring_joints");
    for (double weight : c.joint_motion_weights)
      require(std::isfinite(weight) && weight > 0, "joint_motion_weights", "must be positive and finite");
    auto number = [&node](const std::string& name) {
      double value = read<double>(node, name);
      require(std::isfinite(value), name, "must be finite");
      return value;
    };
#define READ_NUMBER(name) c.name = number(#name)
    READ_NUMBER(approach_min); READ_NUMBER(approach_max); READ_NUMBER(lift_min); READ_NUMBER(lift_max);
    READ_NUMBER(retreat_min); READ_NUMBER(retreat_max); READ_NUMBER(place_lower_min); READ_NUMBER(place_lower_max);
    READ_NUMBER(grasp_pitch); READ_NUMBER(grasp_spin); READ_NUMBER(grasp_depth); READ_NUMBER(grasp_lateral_offset);
    READ_NUMBER(grip_target); READ_NUMBER(open_target); READ_NUMBER(gripper_motion_scaling);
    READ_NUMBER(motion_velocity_scaling); READ_NUMBER(motion_acceleration_scaling);
    READ_NUMBER(transport_velocity_scaling); READ_NUMBER(transport_acceleration_scaling);
    READ_NUMBER(transport_orientation_tolerance); READ_NUMBER(place_height_offset);
    READ_NUMBER(input_wait_timeout_sec); READ_NUMBER(cartesian_step_m); READ_NUMBER(cartesian_scaling_cap);
    READ_NUMBER(connect_timeout_sec); READ_NUMBER(constrained_connect_timeout_sec);
#undef READ_NUMBER
    auto range = [](double lo, double hi, const std::string& name) {
      require(lo > 0 && lo <= hi, name, "require 0 < min <= max");
    };
    range(c.approach_min, c.approach_max, "approach_min/approach_max");
    range(c.lift_min, c.lift_max, "lift_min/lift_max");
    range(c.retreat_min, c.retreat_max, "retreat_min/retreat_max");
    range(c.place_lower_min, c.place_lower_max, "place_lower_min/place_lower_max");
    require(c.place_lower_max <= 0.15, "place_lower_max", "must be <= 0.15 m");
    require(c.grasp_depth >= 0 && c.grasp_depth <= 0.05, "grasp_depth", "must be in [0, 0.05] m");
    require(std::abs(c.grasp_lateral_offset) <= 0.02, "grasp_lateral_offset", "must be within 0.02 m");
    require(c.place_height_offset >= 0 && c.place_height_offset <= 0.05, "place_height_offset", "must be in [0, 0.05] m");
    require(c.transport_orientation_tolerance >= 0.01 && c.transport_orientation_tolerance <= 0.6,
            "transport_orientation_tolerance", "must be in [0.01, 0.6] rad");
    for (auto item : std::vector<std::pair<std::string, double>>{
           {"gripper_motion_scaling", c.gripper_motion_scaling}, {"motion_velocity_scaling", c.motion_velocity_scaling},
           {"motion_acceleration_scaling", c.motion_acceleration_scaling}, {"transport_velocity_scaling", c.transport_velocity_scaling},
           {"transport_acceleration_scaling", c.transport_acceleration_scaling}, {"cartesian_scaling_cap", c.cartesian_scaling_cap}})
      require(item.second > 0 && item.second <= 1, item.first, "must be in (0, 1]");
    for (auto item : std::vector<std::pair<std::string, double>>{
           {"input_wait_timeout_sec", c.input_wait_timeout_sec}, {"cartesian_step_m", c.cartesian_step_m},
           {"connect_timeout_sec", c.connect_timeout_sec}, {"constrained_connect_timeout_sec", c.constrained_connect_timeout_sec}})
      require(item.second > 0, item.first, "must be positive");
    auto direction = [&node](const std::string& name) {
      auto values = read<std::vector<double>>(node, name);
      require(values.size() == 3, name, "must have three components");
      double norm = 0;
      for (double v : values) { require(std::isfinite(v), name, "must be finite"); norm += v * v; }
      require(std::abs(norm - 1.0) <= 1e-6, name, "must be a unit vector");
      return std::array<double, 3>{values[0], values[1], values[2]};
    };
    c.approach_direction = direction("approach_direction"); c.lift_direction = direction("lift_direction");
    c.lower_direction = direction("lower_direction"); c.retreat_direction = direction("retreat_direction");
    c.transport_keep_orientation = read<bool>(node, "transport_keep_orientation");
    const auto count = read<int64_t>(node, "planning_candidates");
    require(count >= 1 && count <= 20, "planning_candidates", "must be in [1, 20]");
    c.planning_candidates = static_cast<int>(count);
    return c;
  }

  void validate_model(const moveit::core::RobotModel& model) const
  {
    for (const auto& entry : model.getURDF()->links_)
      require(!entry.second->visual || entry.second->collision, "collision_model",
              entry.first+" has visual geometry but no collision geometry");
    const auto* arm = model.getJointModelGroup(arm_group);
    const auto* hand = model.getJointModelGroup(hand_group);
    require(arm != nullptr, "arm_group", "group does not exist");
    require(hand != nullptr, "hand_group", "group does not exist");
    const auto* tool = model.getLinkModel(tool_link);
    require(tool && arm->hasLinkModel(tool_link), "tool_link", "must belong to arm_group");
    const auto* eef = model.getEndEffector(end_effector);
    require(eef && eef == hand, "end_effector", "must identify hand_group");
    const auto& parent = eef->getEndEffectorParentGroup();
    require(parent.first == arm_group && parent.second == tool_link, "end_effector", "parent must match arm_group and tool_link");
    const auto* joint = model.getJointModel(hand_joint);
    const auto& active_hand = hand->getActiveJointModels();
    require(joint && joint->getVariableCount() == 1 && active_hand.size() == 1 && active_hand.front() == joint,
            "hand_joint", "hand_group must contain this single active one-DOF joint");
    const auto& bounds = joint->getVariableBounds().front();
    for (auto target : std::vector<std::pair<std::string, double>>{{"open_target", open_target}, {"grip_target", grip_target}})
      require(!bounds.position_bounded_ || (target.second >= bounds.min_position_ && target.second <= bounds.max_position_),
              target.first, "outside hand_joint model limits");
    std::set<std::string> arm_joints;
    for (const auto* arm_joint : arm->getActiveJointModels()) {
      require(arm_joint->getVariableCount() == 1, "scoring_joints", "arm must have single-DOF active joints");
      arm_joints.insert(arm_joint->getName());
    }
    require(arm_joints == std::set<std::string>(scoring_joints.begin(), scoring_joints.end()),
            "scoring_joints", "must exactly cover active arm joints");
    std::map<std::string, double> home;
    require(arm->getVariableDefaultPositions(home_state, home), "home_state", "named arm state does not exist");

    // Find the fixed tool mount below the final active arm joint. Traverse
    // only fixed and hand/mimic joints, never an unrelated movable branch.
    const auto* root = tool;
    while (root->getParentJointModel() && root->getParentJointModel()->getType() == moveit::core::JointModel::FIXED) {
      const auto* next = root->getParentJointModel()->getParentLinkModel();
      if (!next || (next->getParentJointModel() && arm_joints.count(next->getParentJointModel()->getName()))) break;
      root = next;
    }
    std::set<std::string> allowed;
    std::vector<const moveit::core::LinkModel*> queue{root};
    while (!queue.empty()) {
      const auto* link = queue.back(); queue.pop_back();
      if (!link->getShapes().empty()) allowed.insert(link->getName());
      for (const auto* child : link->getChildJointModels()) {
        const auto* mimic = child->getMimic();
        if (child->getType() == moveit::core::JointModel::FIXED || hand->hasJointModel(child->getName()) ||
            (mimic && hand->hasJointModel(mimic->getName()))) queue.push_back(child->getChildLinkModel());
      }
    }
    for (const auto& link : contact_links)
      require(allowed.count(link), "contact_links", link + " has no collision geometry in the gripper subtree");
  }
};
}  // namespace pick_place_mtc
