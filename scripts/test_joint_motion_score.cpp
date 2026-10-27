#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include "pick_place_mtc/joint_motion_score.hpp"

using pick_place_mtc::weighted_joint_motion;

void require(bool condition, const char* message)
{
  if (!condition) throw std::runtime_error(message);
}

template<typename Function>
void invalid_config(Function function)
{
  try { function(); }
  catch (const std::invalid_argument&) { return; }
  throw std::runtime_error("Invalid configuration accepted");
}

int main()
{
  const double inf = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double max = std::numeric_limits<double>::max();
  try {
    require(std::abs(weighted_joint_motion(
      {{{"wrist", "elbow", "hand"}, {0.0, 1.0, nan}},
       {{"elbow"}, {2.0}}, {{"hand"}, {inf}},
       {{"wrist", "elbow"}, {1.0, 3.0}}},
      {"elbow", "wrist"}, {2.0, 3.0}) - 7.0) < 1e-12, "Weighted sparse/reordered joints");
    require(weighted_joint_motion({{{"axis"}, {-3.0}}, {{"axis"}, {3.0}}},
                                  {"axis"}, {2.0}) == 12.0, "Raw delta must not wrap");
    require(weighted_joint_motion({{{"a"}, {0.0}}, {{"hand"}, {1.0}}, {{"a"}, {1.5}}},
                                  {"a"}, {2.0}) == 3.0, "Boundary previous must persist");
    require(weighted_joint_motion({}, {"a"}, {1.0}) == inf, "Empty candidate");
    require(weighted_joint_motion({{{"a"}, {0.0}}}, {"a", "b"}, {1.0, 1.0}) == inf, "Missing joint");
    require(weighted_joint_motion({{{"a", "hand"}, {0.0}}}, {"a"}, {1.0}) == inf, "Short positions");
    require(weighted_joint_motion({{{"a"}, {0.0, 1.0}}}, {"a"}, {1.0}) == inf, "Long positions");
    require(weighted_joint_motion({{{"a", "hand", "hand"}, {0.0, 0.0, 1.0}}},
                                  {"a"}, {1.0}) == inf, "Duplicate non-scoring name");
    for (double value : {nan, inf, -inf}) {
      require(weighted_joint_motion({{{"a"}, {value}}}, {"a"}, {1.0}) == inf, "Nonfinite scoring value");
    }
    require(weighted_joint_motion({{{"a"}, {-max}}, {{"a"}, {max}}},
                                  {"a"}, {1.0}) == inf, "Delta overflow");
    require(weighted_joint_motion({{{"a"}, {0.0}}, {{"a"}, {max}}},
                                  {"a"}, {2.0}) == inf, "Weighted overflow");
    require(weighted_joint_motion({{{"a", "b"}, {0.0, 0.0}}, {{"a", "b"}, {max, max}}},
                                  {"a", "b"}, {1.0, 1.0}) == inf, "Sum overflow");
    invalid_config([] { weighted_joint_motion({}, {}, {}); });
    invalid_config([] { weighted_joint_motion({}, {""}, {1.0}); });
    invalid_config([] { weighted_joint_motion({}, {"a", "a"}, {1.0, 1.0}); });
    invalid_config([] { weighted_joint_motion({}, {"a"}, {}); });
    for (double weight : {0.0, -1.0, nan, inf}) {
      invalid_config([weight] { weighted_joint_motion({}, {"a"}, {weight}); });
    }
    std::cout << "PASS joint motion scoring: configured names, boundaries, malformed data and overflow\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL " << error.what() << '\n';
    return 1;
  }
}
