#pragma once

#include <array>
#include <cmath>
#include <string>
#include <vector>
#include <pick_place_mtc/task_config.hpp>

namespace pick_place_mtc
{
// Each entry constructs a complete MTC task. Existing planning_candidates is
// the number of solutions per entry, never evidence of grasp/path diversity.
struct CandidateProfile
{
  std::string id, planner_id;
  double grasp_yaw = 0., spin_offset = 0., approach_distance = 0., timeout = 10.;
  std::array<double, 3> place_direction, retreat_direction;
  std::vector<double> waypoint;
  int repetitions = 1;

  static std::vector<CandidateProfile> load(const rclcpp::Node& node, const TaskConfig& config)
  {
    std::vector<std::string> ids;
    if (node.has_parameter("candidate_profiles"))
      ids = TaskConfig::read<std::vector<std::string>>(node, "candidate_profiles");
    if (ids.empty()) ids = {"baseline"};
    TaskConfig::require(ids.size() <= 12, "candidate_profiles", "at most 12 bounded profiles");
    std::set<std::string> unique;
    std::vector<CandidateProfile> result;
    for (const auto& id : ids) {
      TaskConfig::require(!id.empty() && unique.insert(id).second, "candidate_profiles", "IDs must be unique and nonempty");
      CandidateProfile p;
      p.id = id; p.planner_id = config.planner_id;
      p.place_direction = config.lower_direction; p.retreat_direction = config.retreat_direction;
      p.timeout = config.connect_timeout_sec;
      auto read = [&](const std::string& key, auto& value) {
        const auto name = "profiles." + id + "." + key;
        if (node.has_parameter(name)) value = TaskConfig::read<std::decay_t<decltype(value)>>(node, name);
      };
      read("planner_id", p.planner_id); read("grasp_yaw_rad", p.grasp_yaw);
      read("spin_offset_rad", p.spin_offset); read("approach_distance_m", p.approach_distance);
      read("planning_timeout_sec", p.timeout); read("transport_waypoint", p.waypoint);
      int64_t repeat = 1; read("repetitions", repeat);
      TaskConfig::require(repeat >= 1 && repeat <= 4, id, "repetitions must be in [1,4]");
      p.repetitions = static_cast<int>(repeat);
      auto direction = [&](const std::string& key, std::array<double, 3>& target) {
        std::vector<double> value(target.begin(), target.end()); read(key, value);
        double norm = 0.;
        for (double v : value) norm += v*v;
        TaskConfig::require(value.size() == 3 && std::isfinite(norm) && std::abs(norm-1.) < 1e-6,
                            id+"."+key, "must be a finite unit vector");
        std::copy(value.begin(), value.end(), target.begin());
      };
      direction("place_direction", p.place_direction); direction("retreat_direction", p.retreat_direction);
      TaskConfig::require(std::isfinite(p.grasp_yaw) && std::isfinite(p.spin_offset) &&
                          std::abs(p.grasp_yaw) <= M_PI && std::abs(p.spin_offset) <= M_PI,
                          id, "angles must be finite within [-pi,pi]");
      TaskConfig::require(std::isfinite(p.approach_distance) && p.approach_distance >= 0. && p.approach_distance <= .2,
                          id, "approach distance must be in [0,.2] m (0 retains task range)");
      TaskConfig::require(std::isfinite(p.timeout) && p.timeout > 0. && p.timeout <= 60., id, "timeout must be in (0,60] s");
      TaskConfig::require(p.waypoint.empty() || p.waypoint.size() == 7, id, "waypoint is empty or xyz+quaternion in motion_frame");
      double qnorm = 0.;
      for (size_t i=0; i<p.waypoint.size(); ++i) {
        TaskConfig::require(std::isfinite(p.waypoint[i]), id, "waypoint must be finite");
        if (i >= 3) qnorm += p.waypoint[i]*p.waypoint[i];
      }
      TaskConfig::require(p.waypoint.empty() || std::abs(qnorm-1.) < 1e-6, id, "waypoint quaternion must be normalized");
      TaskConfig::require(!p.planner_id.empty(), id, "planner_id must be nonempty");
      const auto planner_key = config.pipeline_id+".planner_configs."+p.planner_id+".type";
      TaskConfig::require(node.has_parameter(planner_key), id, "planner_id must identify an explicitly configured planner (no fallback)");
      result.push_back(std::move(p));
    }
    return result;
  }
};
}  // namespace pick_place_mtc
