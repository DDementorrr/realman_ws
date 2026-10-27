#pragma once

#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace pick_place_mtc
{
struct JointSample
{
  std::vector<std::string> names;
  std::vector<double> positions;
};

inline double weighted_joint_motion(const std::vector<JointSample>& samples,
                                    const std::vector<std::string>& joints,
                                    const std::vector<double>& weights)
{
  std::map<std::string, double> configured;
  if (joints.empty() || joints.size() != weights.size()) {
    throw std::invalid_argument("Scoring joints and weights must have matching nonzero sizes");
  }
  for (size_t i = 0; i < joints.size(); ++i) {
    if (joints[i].empty() || !std::isfinite(weights[i]) || weights[i] <= 0.0 ||
        !configured.emplace(joints[i], weights[i]).second) {
      throw std::invalid_argument("Scoring joints must be unique with finite positive weights");
    }
  }
  const double invalid = std::numeric_limits<double>::infinity();
  std::map<std::string, double> previous;
  double score = 0.0;
  for (const auto& sample : samples) {
    if (sample.names.size() != sample.positions.size()) return invalid;
    std::set<std::string> names;
    for (size_t i = 0; i < sample.names.size(); ++i) {
      const auto& name = sample.names[i];
      if (!names.insert(name).second) return invalid;
      auto weight = configured.find(name);
      if (weight == configured.end()) continue;
      const double value = sample.positions[i];
      if (!std::isfinite(value)) return invalid;
      auto before = previous.find(name);
      if (before != previous.end()) {
        // Raw deltas preserve commanded long turns and stage boundaries.
        score += weight->second * std::abs(value - before->second);
        if (!std::isfinite(score)) return invalid;
      }
      previous[name] = value;
    }
  }
  return previous.size() == configured.size() ? score : invalid;
}
}  // namespace pick_place_mtc
