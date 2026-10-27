#pragma once

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <set>
#include <optional>
#include <openssl/sha.h>
#include <yaml-cpp/yaml.h>
#include <rclcpp/serialization.hpp>
#include <moveit/task_constructor/storage.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_env.h>
#include <pick_place_mtc/joint_motion_score.hpp>
#include <pick_place_mtc/task_config.hpp>

namespace pick_place_mtc
{
inline std::string hex(const unsigned char* bytes, size_t count)
{
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (size_t i=0; i<count; ++i) out << std::setw(2) << static_cast<unsigned>(bytes[i]);
  return out.str();
}
inline std::string sha256(const std::string& value)
{
  unsigned char hash[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char*>(value.data()), value.size(), hash);
  return hex(hash, sizeof(hash));
}
template<class Message> std::string cdr_hex(const Message& message)
{
  rclcpp::SerializedMessage serialized;
  rclcpp::Serialization<Message>().serialize_message(&message, &serialized);
  const auto& bytes = serialized.get_rcl_serialized_message();
  return hex(bytes.buffer, bytes.buffer_length);
}
inline std::string file_contents(const std::string& path)
{
  std::ifstream in(path, std::ios::binary);
  if (!in) throw std::runtime_error("Cannot read candidate binding file: " + path);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}
inline void write_archive(const std::string& path, const YAML::Node& report)
{
  if (path.empty()) return;
  const auto parent = std::filesystem::path(path).parent_path();
  if (!parent.empty()) std::filesystem::create_directories(parent);
  std::ofstream out(path+".tmp");
  out << report << '\n'; out.close();
  if (!out) throw std::runtime_error("Cannot persist candidate archive: " + path);
  std::filesystem::rename(path+".tmp", path);
}

struct CandidateAudit
{
  bool accepted = false;
  std::string rejection;
  double clearance = std::numeric_limits<double>::max();
  double length = 0., joint_motion = 0., rotation = 0., deviation = 0.;
  size_t checked_states = 0;
  std::vector<Eigen::Vector3d> tool_path;
  std::set<std::string> regions;
  std::set<std::string> stages;
  std::optional<moveit::core::RobotState> previous;
  const TaskConfig& config;
  double minimum, step, boundary_tolerance;
  double maximum_boundary_delta = 0.;

  CandidateAudit(const TaskConfig& c, double min_clearance, double collision_step, double boundary = .001)
    : config(c), minimum(min_clearance), step(collision_step), boundary_tolerance(boundary) {}

  void fail(const std::string& reason) { if (rejection.empty()) rejection = reason; }

  void state(const planning_scene::PlanningSceneConstPtr& scene, moveit::core::RobotState current)
  {
    current.update(); ++checked_states;
    for (const auto* joint : current.getRobotModel()->getActiveJointModels())
      if (!current.satisfiesPositionBounds(joint,1e-7)) fail("joint_position_limit");
    for (const auto* joint : current.getRobotModel()->getActiveJointModels()) {
      for (const auto& name : joint->getVariableNames()) {
        const auto& b = current.getRobotModel()->getVariableBounds(name);
        const double q = current.getVariablePosition(name);
        if (!std::isfinite(q)) fail("nonfinite_position");
        if (current.hasVelocities()) {
          const double v = current.getVariableVelocity(name);
          if (!std::isfinite(v) || !b.velocity_bounded_ || std::abs(v) > b.max_velocity_+1e-6)
            fail("joint_velocity_limit:"+name);
        }
        if (current.hasAccelerations()) {
          const double a = current.getVariableAcceleration(name);
          if (!std::isfinite(a) || !b.acceleration_bounded_ || std::abs(a) > b.max_acceleration_+1e-6)
            fail("joint_acceleration_limit:"+name);
        }
      }
    }
    collision_detection::CollisionRequest request;
    collision_detection::CollisionResult result;
    scene->checkCollision(request, result, current);
    if (result.collision) fail("collision");
    // Includes the entire robot, camera and attached payload. The stage's
    // original ACM permits intentional grasp/support contact only.
    collision_detection::DistanceRequest distance;
    distance.acm = &scene->getAllowedCollisionMatrix();
    collision_detection::DistanceResult world;
    scene->getCollisionEnv()->distanceRobot(distance, world, current);
    const double d = world.minimum_distance.distance;
    if (!std::isfinite(d) || d < minimum) fail("minimum_world_clearance");
    clearance = std::min(clearance, d);
  }

  void visit(const moveit::task_constructor::SolutionBase& solution)
  {
    namespace mtc = moveit::task_constructor;
    if (auto wrapped = dynamic_cast<const mtc::WrappedSolution*>(&solution)) { visit(*wrapped->wrapped()); return; }
    if (auto sequence = dynamic_cast<const mtc::SolutionSequence*>(&solution)) {
      for (const auto* child : sequence->solutions()) visit(*child);
      return;
    }
    const auto* part = dynamic_cast<const mtc::SubTrajectory*>(&solution);
    if (!part || !part->start() || !part->end()) { fail("unsupported_or_incomplete_solution"); return; }
    const auto stage = part->creator() ? part->creator()->name() : std::string();
    stages.insert(stage);
    const auto trajectory = part->trajectory();
    auto compose = [&](const planning_scene::PlanningSceneConstPtr& scene,
                       const moveit::core::RobotState* point = nullptr) {
      moveit::core::RobotState q(scene->getCurrentState());
      std::map<std::string, double> positions;
      if (previous) for (const auto* joint : q.getRobotModel()->getActiveJointModels())
        for (const auto& name : joint->getVariableNames()) positions[name] = previous->getVariablePosition(name);
      if (point) for (const auto& name : trajectory->getGroup()->getVariableNames())
        positions[name] = point->getVariablePosition(name);
      q.setVariablePositions(positions);
      std::vector<double> zeros(q.getVariableCount(),0.);
      q.setVariableVelocities(zeros.data()); q.setVariableAccelerations(zeros.data());
      if (point) for (const auto& name : trajectory->getGroup()->getVariableNames()) {
        if (point->hasVelocities()) q.setVariableVelocity(name, point->getVariableVelocity(name));
        if (point->hasAccelerations()) q.setVariableAcceleration(name, point->getVariableAcceleration(name));
      }
      q.update(); return q;
    };
    if (!trajectory || trajectory->getWayPointCount() == 0) {
      auto q = compose(part->end()->scene());
      state(part->end()->scene(), q); previous = q; return;
    }
    for (size_t i=0; i<trajectory->getWayPointCount(); ++i) {
      const auto& raw = trajectory->getWayPoint(i);
      if (!raw.hasVelocities() || !raw.hasAccelerations()) fail("missing_time_parameterization");
      auto q = compose(part->start()->scene(), &raw);
      if (!q.hasVelocities() || !q.hasAccelerations()) fail("missing_time_parameterization");
      const double dt = trajectory->getWayPointDurationFromPrevious(i);
      if (!std::isfinite(dt) || dt < 0.) fail("invalid_duration");
      if (previous) {
        double delta = 0.; std::string largest_joint;
        for (const auto* joint : q.getRobotModel()->getActiveJointModels()) {
          for (const auto& name : joint->getVariableNames()) {
            const double dq = std::abs(q.getVariablePosition(name)-previous->getVariablePosition(name));
            if (dq > delta) { delta = dq; largest_joint = name; }
            const auto& b = q.getRobotModel()->getVariableBounds(name);
            if (i > 0 && dt > 0. && dq/dt > b.max_velocity_+1e-5) fail("finite_difference_velocity:"+name);
            if (i > 0 && dt > 0. && previous->hasVelocities() && q.hasVelocities() &&
                std::abs(q.getVariableVelocity(name)-previous->getVariableVelocity(name))/dt > b.max_acceleration_+1e-4)
              fail("finite_difference_acceleration:"+name);
          }
        }
        if (i == 0) maximum_boundary_delta = std::max(maximum_boundary_delta, delta);
        if ((i == 0 && delta > boundary_tolerance) || (i > 0 && dt == 0. && delta > 1e-8)) fail("discontinuous_stage_boundary:"+stage+":"+largest_joint+":"+std::to_string(delta));
        const size_t count = std::max<size_t>(1, static_cast<size_t>(std::ceil(delta/step)));
        for (size_t j=1; j<count; ++j) {
          moveit::core::RobotState sample(q);
          previous->interpolate(q, double(j)/count, sample);
          state(part->start()->scene(), std::move(sample));
        }
      }
      state(part->start()->scene(), q);
      const auto& transform = q.getGlobalLinkTransform(config.tool_link);
      if (!tool_path.empty()) length += (transform.translation()-tool_path.back()).norm();
      if (previous) rotation += Eigen::Quaterniond(previous->getGlobalLinkTransform(config.tool_link).rotation())
                                  .angularDistance(Eigen::Quaterniond(transform.rotation()));
      tool_path.push_back(transform.translation());
      // Fixed world cells, 10 cm resolution; report measured route occupancy.
      const auto& p = tool_path.back();
      regions.insert(std::to_string(static_cast<int>(std::floor(p.x()/.1)))+","+
                     std::to_string(static_cast<int>(std::floor(p.y()/.1)))+","+
                     std::to_string(static_cast<int>(std::floor(p.z()/.1))));
      previous = q;
    }
    auto end = compose(part->end()->scene());
    state(part->end()->scene(), end); previous = end;
  }

  YAML::Node finish(const moveit::task_constructor::SolutionBase& solution,
                    const moveit_task_constructor_msgs::msg::Solution& message, double seconds)
  {
    visit(solution);
    for (const auto* name : {"Approach (Cartesian)", "Close Gripper", "Attach in PlanningScene", "Lift (Cartesian)",
                            "Transport (OMPL)", "Place (Cartesian)", "Open Gripper", "Detach in PlanningScene",
                            "Retreat (Cartesian)", "return Home"})
      if (!stages.count(name)) fail(std::string("incomplete_task:")+name);
    std::vector<JointSample> samples;
    for (const auto& part : message.sub_trajectory)
      for (const auto& point : part.trajectory.joint_trajectory.points)
        samples.push_back({part.trajectory.joint_trajectory.joint_names, point.positions});
    joint_motion = weighted_joint_motion(samples, config.scoring_joints, config.joint_motion_weights);
    if (!std::isfinite(joint_motion)) fail("invalid_joint_score");
    if (tool_path.size() < 2) fail("empty_tool_path");
    else {
      const Eigen::Vector3d start = tool_path.front(), end = tool_path.back(), line = end-start;
      for (const auto& p : tool_path) {
        const double t = line.squaredNorm() > 1e-12 ? std::clamp((p-start).dot(line)/line.squaredNorm(), 0., 1.) : 0.;
        deviation += (p-(start+t*line)).norm();
      }
      deviation /= tool_path.size();
    }
    accepted = rejection.empty();
    YAML::Node out;
    out["accepted"] = accepted; out["rejection"] = rejection;
    out["minimum_world_clearance_m"] = clearance == std::numeric_limits<double>::max() ? 0. : clearance;
    out["collision_checked_states"] = checked_states;
    out["maximum_stage_boundary_delta_rad"] = maximum_boundary_delta;
    out["tool_path_length_m"] = length; out["weighted_joint_travel_rad"] = joint_motion;
    out["planning_time_sec"] = seconds; out["tool_orientation_travel_rad"] = rotation;
    out["tool_chord_deviation_m"] = deviation;
    for (const auto& region : regions) out["traversed_regions_0_1m"].push_back(region);
    // Arc-length resampling: differing planner waypoint counts must not
    // masquerade as route diversity. A stationary path stays constant.
    std::vector<double> cumulative{0.};
    for (size_t i=1; i<tool_path.size(); ++i)
      cumulative.push_back(cumulative.back()+(tool_path[i]-tool_path[i-1]).norm());
    if (!tool_path.empty()) for (size_t i=0; i<32; ++i) {
      const double distance = cumulative.back()*i/31.;
      size_t k = std::min<size_t>(std::lower_bound(cumulative.begin(),cumulative.end(),distance)-cumulative.begin(),tool_path.size()-1);
      size_t j = k > 0 ? k-1 : k;
      const double span = cumulative[k]-cumulative[j];
      const double t = span > 1e-12 ? (distance-cumulative[j])/span : 0.;
      Eigen::Vector3d p = tool_path[j]+t*(tool_path[k]-tool_path[j]);
      out["path_signature"].push_back(std::vector<double>{p.x(),p.y(),p.z()});
    }
    if (accepted) {
      // Fixed references and weights; no per-run min/max or clipping. Values
      // above 1 remain above 1. Clearance can never offset a hard rejection.
      out["normalized"] = std::vector<double>{length/2., joint_motion/100., seconds/30.,
        .01/std::max(clearance, minimum), deviation/.5, rotation/(2.*M_PI)};
      const std::vector<double> weights{.25,.30,.05,.20,.10,.10};
      double score = 0.;
      const auto normalized = out["normalized"].as<std::vector<double>>();
      for (size_t i=0; i<weights.size(); ++i) score += weights[i]*normalized[i];
      out["score"] = score;
    }
    return out;
  }
};
}  // namespace pick_place_mtc
