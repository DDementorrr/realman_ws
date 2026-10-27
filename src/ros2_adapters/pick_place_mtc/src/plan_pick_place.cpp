#include <algorithm>
#include <atomic>
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
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/task.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <moveit_task_constructor_msgs/msg/task_description.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <pick_place_mtc/joint_motion_score.hpp>
#include <pick_place_mtc/task_config.hpp>
#include <pick_place_mtc/candidate_profile.hpp>
#include <pick_place_mtc/candidate_audit.hpp>

namespace mtc = moveit::task_constructor;
using namespace std::chrono_literals;

class PickPlacePlanner
{
public:
  PickPlacePlanner()
    : node_(std::make_shared<rclcpp::Node>(
          "plan_pick_place", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)))
  {
    config_ = pick_place_mtc::TaskConfig::load(*node_);
    joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        feedback_ = *message; feedback_arrived_ = std::chrono::steady_clock::now();
      });
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
    candidate_revision_pub_ = node_->create_publisher<std_msgs::msg::UInt64>(
      "/pick_place/candidate_scene_revision", rclcpp::QoS(1).reliable().transient_local());
    scene_revision_sub_ = node_->create_subscription<std_msgs::msg::UInt64>(
      "/perception/scene_revision", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::UInt64::ConstSharedPtr msg) {
        scene_revision_.store(msg->data);
        scene_revision_seen_.store(true);
      });
    scene_ready_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      "/perception/scene_ready", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) { scene_ready_.store(msg->data); });
    parameter_guard_ = node_->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>&) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = !planning_active_.load();
      result.reason = result.successful ? "" : "candidate inputs are immutable while planning; restart for a new configuration";
      return result;
    });
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base() const
  {
    return node_->get_node_base_interface();
  }

  bool run()
  {
    task_.loadRobotModel(node_);
    config_.validate_model(*task_.getRobotModel());
    model_ = task_.getRobotModel();
    geometry_msgs::msg::PoseStamped place_pose, target_pose;
    bool scene_ready = false;
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(config_.input_wait_timeout_sec);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      bool poses_ready = false;
      {
        std::lock_guard<std::mutex> lock(poses_mutex_);
        if (object_pose_ && place_pose_) {
          place_pose = *place_pose_;
          target_pose = *object_pose_;
          poses_ready = true;
        }
      }
      scene_ready = poses_ready && feedback_current() && has_known_scene(deadline) &&
        (!scene_revision_seen_.load() || scene_ready_.load());
      if (scene_ready) {
        break;
      }
      std::this_thread::sleep_until(std::min(deadline,
        decltype(deadline)(std::chrono::steady_clock::now() + 50ms)));
    }
    if (!scene_ready) {
      RCLCPP_ERROR(node_->get_logger(), "input_wait_timeout_sec: Timed out waiting for known poses and MoveIt collision scene");
      return false;
    }
    const bool revision_bound = scene_revision_seen_.load();
    const auto candidate_revision = scene_revision_.load();

    auto scene_request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    scene_request->components.components = 1023;  // all Humble PlanningSceneComponents
    auto scene_future = scene_client_->async_send_request(scene_request);
    if (scene_future.wait_for(5s) != std::future_status::ready) {
      scene_client_->remove_pending_request(scene_future);
      throw std::runtime_error("candidate initial scene query timed out");
    }
    const auto initial_message = scene_future.get()->scene;
    auto initial_scene = std::make_shared<planning_scene::PlanningScene>(model_);
    if (!initial_scene->setPlanningSceneMsg(initial_message))
      throw std::runtime_error("candidate initial scene import failed");
    auto number = [this](const std::string& name, double fallback) {
      const double value = node_->has_parameter(name) ? pick_place_mtc::TaskConfig::read<double>(*node_, name) : fallback;
      if (!std::isfinite(value) || value <= 0.) throw std::invalid_argument(name+": must be positive and finite");
      return value;
    };
    const double min_clearance = number("candidate_min_clearance_m", .001);
    const double collision_step = number("candidate_collision_step_rad", .01);
    const double boundary_tolerance = number("candidate_boundary_tolerance_rad", .001);
    if (boundary_tolerance > .001) throw std::invalid_argument("candidate boundary tolerance must be <= .001 rad");
    const double start_tolerance = number("candidate_start_tolerance_rad", .01);
    if (!feedback_matches(initial_scene->getCurrentState(),start_tolerance))
      throw std::runtime_error("candidate initial state does not match fresh joint feedback");
    if (collision_step > .01 || start_tolerance > .01)
      throw std::invalid_argument("candidate collision step/start tolerance cannot exceed .01 rad");
    const auto run_id = pick_place_mtc::sha256(std::to_string(std::chrono::system_clock::now().time_since_epoch().count())+
                                             std::to_string(node_->now().nanoseconds())).substr(0,24);
    std::string archive_path;
    if (node_->has_parameter("candidate_archive")) node_->get_parameter("candidate_archive", archive_path);
    if (archive_path.empty()) archive_path = "/tmp/pick-place-candidates-"+run_id+".yaml";
    if (std::filesystem::exists(archive_path)) throw std::runtime_error("candidate archive must be a new path");
    YAML::Node archive;
    archive["schema"] = "pick_place_candidates/v1"; archive["run_id"] = run_id;
    archive["rng_seed"] = YAML::Node();
    archive["rng_policy"] = "installed PipelinePlanner exposes no per-request seed; process RNG is not controlled";
    archive["scoring"]["units"] = std::vector<std::string>{"m", "weighted_rad", "s", "m", "m", "rad"};
    archive["scoring"]["functions"] = std::vector<std::string>{"x/2", "x/100", "x/30", "0.01/x", "x/0.5", "x/(2*pi)"};
    archive["scoring"]["normalization"] = std::vector<double>{2.,100.,30.,.01,.5,2.*M_PI};
    archive["scoring"]["weights"] = std::vector<double>{.25,.30,.05,.20,.10,.10};
    archive["scoring"]["boundary_policy"] = "hard reject first; fixed references; no clipping; lower is better; first wins exact ties";
    archive["binding"]["scene_cdr_hex"] = pick_place_mtc::cdr_hex(initial_message);
    archive["binding"]["scene_revision_bound"] = revision_bound;
    archive["binding"]["scene_revision"] = candidate_revision;
    archive["binding"]["start_tolerance_rad"] = start_tolerance;
    archive["binding"]["minimum_clearance_m"] = min_clearance;
    archive["binding"]["collision_step_rad"] = collision_step;
    archive["binding"]["boundary_tolerance_rad"] = boundary_tolerance;
    archive["binding"]["object_id"] = config_.object_id;
    archive["binding"]["target_frame"] = target_pose.header.frame_id;
    archive["binding"]["target_xyz_m"] = std::vector<double>{target_pose.pose.position.x,target_pose.pose.position.y,target_pose.pose.position.z};
    archive["binding"]["target_xyzw"] = std::vector<double>{target_pose.pose.orientation.x,target_pose.pose.orientation.y,target_pose.pose.orientation.z,target_pose.pose.orientation.w};
    archive["binding"]["place_frame"] = place_pose.header.frame_id;
    archive["binding"]["place_xyz_m"] = std::vector<double>{place_pose.pose.position.x,place_pose.pose.position.y,place_pose.pose.position.z};
    archive["binding"]["place_xyzw"] = std::vector<double>{place_pose.pose.orientation.x,place_pose.pose.orientation.y,place_pose.pose.orientation.z,place_pose.pose.orientation.w};
    archive["binding"]["urdf_sha256"] = pick_place_mtc::sha256(node_->get_parameter("robot_description").as_string());
    archive["binding"]["srdf_sha256"] = pick_place_mtc::sha256(node_->get_parameter("robot_description_semantic").as_string());
    archive["binding"]["target_cdr_hex"] = pick_place_mtc::cdr_hex(target_pose);
    archive["binding"]["place_cdr_hex"] = pick_place_mtc::cdr_hex(place_pose);
    for (const auto* joint : model_->getActiveJointModels())
      for (const auto& name : joint->getVariableNames())
        archive["binding"]["start_positions"][name] = initial_scene->getCurrentState().getVariablePosition(name);
    if (node_->has_parameter("candidate_binding_files"))
      for (const auto& item : YAML::Load(pick_place_mtc::TaskConfig::read<std::string>(*node_, "candidate_binding_files"))) {
        const auto path = item.as<std::string>();
        archive["binding"]["files"][path] = pick_place_mtc::sha256(pick_place_mtc::file_contents(path));
      }
    // Bind the actual external robot collision assets, not just URDF URIs.
    for (const auto& entry : model_->getURDF()->links_) {
      for (const auto& collision : entry.second->collision_array) {
        if (!collision->geometry || collision->geometry->type != urdf::Geometry::MESH) continue;
        auto path = std::static_pointer_cast<urdf::Mesh>(collision->geometry)->filename;
        if (path.find("package://") == 0) {
          const auto split = path.find('/',10);
          if (split == std::string::npos) throw std::runtime_error("invalid collision mesh package URI");
          path = ament_index_cpp::get_package_share_directory(path.substr(10,split-10))+path.substr(split);
        } else if (path.find("file://") == 0) path = path.substr(7);
        if (!std::filesystem::path(path).is_absolute()) throw std::runtime_error("collision mesh requires an absolute or package URI");
        archive["binding"]["files"][path] = pick_place_mtc::sha256(pick_place_mtc::file_contents(path));
      }
    }
    // All task and model parameters are bound, including baseline tuning.
    std::map<std::string, rclcpp::Parameter> original_parameters;
    for (const auto& name : node_->list_parameters({},100).names) {
      if (name.find("qos_overrides.") == 0 || name.find("start_type") == 0) continue;
      try {
        const auto value = node_->get_parameter(name);
        original_parameters.emplace(name, value);
        archive["binding"]["planner_parameters"][name] = pick_place_mtc::cdr_hex(value.to_parameter_msg());
      } catch (const rclcpp::exceptions::ParameterUninitializedException&) {
        // Kinematics plugins declare optional empty arrays on Humble. They
        // have no effective value to bind; no fallback replaces a real value.
        archive["binding"]["uninitialized_parameters"].push_back(name);
      }
    }
    mtc::SolutionBaseConstPtr selected;
    double best = std::numeric_limits<double>::infinity();
    size_t selected_index = 0, selected_task = 0;
    auto& tasks = tasks_;
    for (const auto& profile : pick_place_mtc::CandidateProfile::load(*node_, config_)) {
      for (int repeat=0; repeat<profile.repetitions; ++repeat) {
        const auto started = std::chrono::steady_clock::now();
        auto owned_task = std::make_unique<mtc::Task>(make_task(place_pose, profile, initial_scene));
        auto& working = *owned_task;
        working.setTimeout(profile.timeout);
        try { working.init(); working.plan(config_.planning_candidates); }
        catch (const std::exception& error) {
          YAML::Node failure;
          failure["candidate_id"] = run_id+"/"+profile.id+"/"+std::to_string(repeat)+"/failed";
          failure["profile"] = profile.id; failure["accepted"] = false;
          failure["rejection"] = std::string("planning_exception:")+error.what();
          archive["candidates"].push_back(failure);
        }
        const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
        if (working.solutions().empty()) {
          YAML::Node failure;
          failure["candidate_id"] = run_id+"/"+profile.id+"/"+std::to_string(repeat)+"/no_solution";
          failure["profile"] = profile.id; failure["accepted"] = false;
          failure["rejection"] = "no_complete_mtc_solution"; failure["planning_time_sec"] = seconds;
          archive["candidates"].push_back(failure);
        }
        size_t index = 0;
        for (const auto& candidate : working.solutions()) {
          moveit_task_constructor_msgs::msg::Solution message;
          candidate->toMsg(message, &working.introspection());
          pick_place_mtc::CandidateAudit audit(config_, min_clearance, collision_step, boundary_tolerance);
          YAML::Node record = audit.finish(*candidate, message, seconds);
          record["candidate_id"] = run_id+"/"+profile.id+"/"+std::to_string(repeat)+"/"+std::to_string(index++);
          record["profile"] = profile.id; record["repetition"] = repeat;
          record["grasp_yaw_rad"] = profile.grasp_yaw; record["spin_rad"] = config_.grasp_spin+profile.spin_offset;
          record["approach_distance_m"] = profile.approach_distance;
          record["transport_waypoint"] = profile.waypoint; record["planner_id"] = profile.planner_id;
          record["planning_budget_sec"] = profile.timeout;
          record["place_direction"] = std::vector<double>(profile.place_direction.begin(),profile.place_direction.end());
          record["retreat_direction"] = std::vector<double>(profile.retreat_direction.begin(),profile.retreat_direction.end());
          if (record["accepted"].as<bool>() && record["score"].as<double>() < best) {
            best = record["score"].as<double>(); selected = candidate;
            selected_index = archive["candidates"].size(); selected_task = tasks.size();
          }
          RCLCPP_INFO(node_->get_logger(), "Candidate %s accepted=%d reason=%s clearance=%.6f checked=%zu",
            record["candidate_id"].as<std::string>().c_str(), audit.accepted, audit.rejection.c_str(), audit.clearance, audit.checked_states);
          archive["candidates"].push_back(record);
        }
        tasks.push_back(std::move(owned_task));
        pick_place_mtc::write_archive(archive_path, archive);
        if (revision_bound && (!scene_ready_.load() || scene_revision_.load() != candidate_revision))
          throw std::runtime_error("scene changed during candidate generation; archive retained, nothing published");
      }
    }
    std::set<std::string> accepted_directions, accepted_regions;
    for (size_t i=0; i<archive["candidates"].size(); ++i) {
      const auto a = archive["candidates"][i];
      if (!a["accepted"].as<bool>()) continue;
      accepted_directions.insert(std::to_string(a["grasp_yaw_rad"].as<double>())+","+std::to_string(a["spin_rad"].as<double>()));
      for (const auto& cell : a["traversed_regions_0_1m"]) accepted_regions.insert(cell.as<std::string>());
      for (size_t j=i+1; j<archive["candidates"].size(); ++j) {
        const auto b = archive["candidates"][j];
        if (!b["accepted"].as<bool>()) continue;
        double squared = 0.;
        for (size_t k=0; k<32; ++k) for (size_t axis=0; axis<3; ++axis) {
          const double delta = a["path_signature"][k][axis].as<double>()-b["path_signature"][k][axis].as<double>();
          squared += delta*delta;
        }
        std::set<std::string> union_cells, cells_a;
        for (const auto& cell : a["traversed_regions_0_1m"]) union_cells.insert(cell.as<std::string>());
        cells_a = union_cells; size_t intersection = 0;
        for (const auto& cell : b["traversed_regions_0_1m"]) {
          intersection += cells_a.count(cell.as<std::string>()); union_cells.insert(cell.as<std::string>());
        }
        YAML::Node pair;
        pair["a"] = a["candidate_id"]; pair["b"] = b["candidate_id"];
        pair["tool_path_rms_m"] = std::sqrt(squared/32.);
        pair["region_jaccard_similarity"] = double(intersection)/union_cells.size();
        archive["diversity"]["pairs"].push_back(pair);
      }
    }
    archive["diversity"]["accepted_grasp_direction_spin_count"] = accepted_directions.size();
    archive["diversity"]["accepted_region_count"] = accepted_regions.size();
    archive["diversity"]["policy"] = "32 arc-length samples over full task; 10cm world cells; repeated OMPL draws alone prove no diversity";
    pick_place_mtc::write_archive(archive_path, archive);
    if (!selected) {
      RCLCPP_ERROR(node_->get_logger(), "No complete candidate passed hard constraints; archive: %s", archive_path.c_str());
      return false;
    }
    auto& chosen_task = *tasks.at(selected_task);
    // Re-read the actual scene and state before publication, even without a
    // revision producer. Consumer performs a fresh independent comparison.
    auto final_future = scene_client_->async_send_request(scene_request);
    if (final_future.wait_for(5s) != std::future_status::ready) throw std::runtime_error("final scene binding query timed out");
    const auto final_scene = final_future.get()->scene;
    if (!feedback_matches(initial_scene->getCurrentState(),start_tolerance))
      throw std::runtime_error("candidate publication state does not match fresh joint feedback");
    const auto& start = archive["binding"]["start_positions"];
    for (size_t i=0; i<final_scene.robot_state.joint_state.name.size(); ++i) {
      const auto& name = final_scene.robot_state.joint_state.name[i];
      if (start[name] && std::abs(start[name].as<double>()-final_scene.robot_state.joint_state.position.at(i)) > start_tolerance)
        throw std::runtime_error("robot moved during candidate generation");
    }
    if (revision_bound && (!scene_ready_.load() || scene_revision_.load() != candidate_revision))
      throw std::runtime_error("scene revision changed before publication");
    for (const auto& entry : original_parameters)
      if (node_->get_parameter(entry.first).to_parameter_msg() != entry.second.to_parameter_msg())
        throw std::runtime_error("candidate planning/model parameter changed during generation: "+entry.first);
    auto comparable_scene = [](moveit_msgs::msg::PlanningScene scene) {
      for (auto& object : scene.world.collision_objects) object.header.stamp = builtin_interfaces::msg::Time();
      scene.world.octomap.header.stamp = builtin_interfaces::msg::Time();
      scene.world.octomap.octomap.header.stamp = builtin_interfaces::msg::Time();
      std::sort(scene.world.collision_objects.begin(), scene.world.collision_objects.end(),
                [](const auto& a, const auto& b) { return a.id < b.id; });
      for (auto& tf : scene.fixed_frame_transforms) tf.header.stamp = builtin_interfaces::msg::Time();
      std::sort(scene.fixed_frame_transforms.begin(), scene.fixed_frame_transforms.end(),
                [](const auto& a, const auto& b) { return a.child_frame_id < b.child_frame_id; });
      return scene;
    };
    const auto initial_geometry = comparable_scene(initial_message), final_geometry = comparable_scene(final_scene);
    if (initial_geometry.world != final_geometry.world ||
        initial_geometry.allowed_collision_matrix != final_geometry.allowed_collision_matrix ||
        initial_geometry.fixed_frame_transforms != final_geometry.fixed_frame_transforms ||
        initial_geometry.link_padding != final_geometry.link_padding || initial_geometry.link_scale != final_geometry.link_scale ||
        initial_geometry.robot_state.attached_collision_objects != final_geometry.robot_state.attached_collision_objects)
      throw std::runtime_error("candidate scene/model transform changed during generation");
    for (const auto& name : node_->list_parameters({},100).names) {
      if (name.find("qos_overrides.") == 0) continue;
      try {
        const auto value = pick_place_mtc::cdr_hex(node_->get_parameter(name).to_parameter_msg());
        archive["binding"]["planner_parameters"][name] = value;
      } catch (const rclcpp::exceptions::ParameterUninitializedException&) {}
    }
    for (const auto& entry : archive["binding"]["files"])
      if (pick_place_mtc::sha256(pick_place_mtc::file_contents(entry.first.as<std::string>())) != entry.second.as<std::string>())
        throw std::runtime_error("candidate configuration/calibration file changed during planning");
    chosen_task.introspection().publishSolution(*selected);
    moveit_task_constructor_msgs::msg::TaskDescription description;
    chosen_task.introspection().fillTaskDescription(description);
    moveit_task_constructor_msgs::msg::Solution solution;
    selected->toMsg(solution, &chosen_task.introspection());
    archive["selected_index"] = selected_index;
    archive["selected_id"] = archive["candidates"][selected_index]["candidate_id"];
    archive["binding"]["solution_cdr_hex"] = pick_place_mtc::cdr_hex(solution);
    archive["binding"]["description_cdr_hex"] = pick_place_mtc::cdr_hex(description);
    archive["binding"]["solution_sha256"] = pick_place_mtc::sha256(archive["binding"]["solution_cdr_hex"].as<std::string>());
    pick_place_mtc::write_archive(archive_path, archive);
    std_msgs::msg::String metrics;
    metrics.data = YAML::Dump(archive);
    metrics_pub_->publish(metrics);
    description_pub_->publish(description);
    solution_pub_->publish(solution);
    RCLCPP_INFO(node_->get_logger(), "Selected candidate %s score=%.6f; archive: %s",
      archive["selected_id"].as<std::string>().c_str(), best, archive_path.c_str());
    if (revision_bound) {
      std_msgs::msg::UInt64 revision;
      revision.data = candidate_revision;
      candidate_revision_pub_->publish(revision);
    }
    std_msgs::msg::Bool ready;
    ready.data = true;
    plan_ready_pub_->publish(ready);
    RCLCPP_INFO(node_->get_logger(), "Complete MTC solution published; execution is a later stage gate");
    planning_active_.store(false);
    return true;
  }

private:
  bool feedback_current()
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    if (!feedback_) return false;
    const double age = (node_->now()-rclcpp::Time(feedback_->header.stamp)).seconds();
    return age >= -.05 && age <= 1. &&
      std::chrono::duration<double>(std::chrono::steady_clock::now()-feedback_arrived_).count() <= 1.;
  }

  bool feedback_matches(const moveit::core::RobotState& state, double tolerance)
  {
    if (!feedback_current()) return false;
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    if (!feedback_ || feedback_->name.size() != feedback_->position.size()) return false;
    std::map<std::string,double> positions;
    for (size_t i=0; i<feedback_->name.size(); ++i)
      if (!std::isfinite(feedback_->position[i]) || !positions.emplace(feedback_->name[i],feedback_->position[i]).second)
        return false;
    for (const auto* joint : state.getRobotModel()->getActiveJointModels())
      for (const auto& name : joint->getVariableNames())
        if (!positions.count(name) || std::abs(positions.at(name)-state.getVariablePosition(name)) > tolerance) return false;
    return true;
  }

  geometry_msgs::msg::Vector3Stamped direction(const std::array<double, 3>& values) const
  {
    geometry_msgs::msg::Vector3Stamped result;
    result.header.frame_id = config_.motion_frame;
    result.vector.x = values[0]; result.vector.y = values[1]; result.vector.z = values[2];
    return result;
  }

  template <typename Deadline>
  bool has_known_scene(const Deadline& deadline)
  {
    if (!scene_client_->service_is_ready()) {
      return false;
    }
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components = moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;
    auto future = scene_client_->async_send_request(request);
    if (future.wait_until(std::min(deadline,
          Deadline(std::chrono::steady_clock::now() + 500ms))) != std::future_status::ready) {
      scene_client_->remove_pending_request(future);
      return false;
    }
    bool object_found = false;
    bool table_found = false;
    for (const auto& object : future.get()->scene.world.collision_objects) {
      object_found |= object.id == config_.object_id;
      table_found |= object.id == config_.support_id;
    }
    return object_found && table_found;
  }

  mtc::Task make_task(const geometry_msgs::msg::PoseStamped& place_pose,
                      const pick_place_mtc::CandidateProfile& profile,
                      const planning_scene::PlanningScenePtr& initial_scene)
  {
    const auto& arm = config_.arm_group;
    const auto& hand = config_.hand_group;
    const auto& tool = config_.tool_link;
    const auto& object = config_.object_id;
    const auto& table = config_.support_id;
    const auto& gripper_links = config_.contact_links;
    const double grasp_spin = config_.grasp_spin+profile.spin_offset;
    const double grasp_depth = config_.grasp_depth;
    const double grasp_lateral_offset = config_.grasp_lateral_offset;
    const double grip_target = config_.grip_target;
    const double open_target = config_.open_target;
    const double gripper_motion_scaling = config_.gripper_motion_scaling;
    const double motion_velocity_scaling = config_.motion_velocity_scaling;
    const double motion_acceleration_scaling = config_.motion_acceleration_scaling;
    const double transport_velocity_scaling = config_.transport_velocity_scaling;
    const double transport_acceleration_scaling = config_.transport_acceleration_scaling;
    const bool transport_keep_orientation = config_.transport_keep_orientation;
    const double transport_orientation_tolerance = config_.transport_orientation_tolerance;
    const double place_lower_min = config_.place_lower_min;
    const double place_lower_max = config_.place_lower_max;
    const double place_height_offset = config_.place_height_offset;

    mtc::Task task;
    task.stages()->setName(config_.task_name);
    task.setRobotModel(model_);
    task.setProperty("group", std::string(arm));
    task.setProperty("eef", config_.end_effector);
    task.setProperty("ik_frame", std::string(tool));
    auto ompl = std::make_shared<mtc::solvers::PipelinePlanner>(node_, config_.pipeline_id);
    ompl->setPlannerId(profile.planner_id);
    ompl->setMaxVelocityScalingFactor(motion_velocity_scaling);
    ompl->setMaxAccelerationScalingFactor(motion_acceleration_scaling);
    auto transport_ompl = std::make_shared<mtc::solvers::PipelinePlanner>(node_, config_.pipeline_id);
    transport_ompl->setPlannerId(profile.planner_id);
    transport_ompl->setMaxVelocityScalingFactor(transport_velocity_scaling);
    transport_ompl->setMaxAccelerationScalingFactor(transport_acceleration_scaling);
    auto joints = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    joints->setMaxVelocityScalingFactor(gripper_motion_scaling);
    joints->setMaxAccelerationScalingFactor(gripper_motion_scaling);
    auto cartesian = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian->setStepSize(config_.cartesian_step_m);
    cartesian->setMaxVelocityScalingFactor(std::min(config_.cartesian_scaling_cap, motion_velocity_scaling));
    cartesian->setMaxAccelerationScalingFactor(std::min(config_.cartesian_scaling_cap, motion_acceleration_scaling));

    auto initial = std::make_unique<mtc::stages::FixedState>("current state");
    initial->setState(initial_scene);
    task.add(std::move(initial));
    {
      auto open = std::make_unique<mtc::stages::MoveTo>("open gripper", joints);
      open->setGroup(hand);
      open->setGoal(std::map<std::string, double>{{config_.hand_joint, open_target}});
      task.add(std::move(open));
    }
    auto contact = std::make_unique<mtc::stages::ModifyPlanningScene>("allow gripper contact");
    contact->allowCollisions(object, gripper_links, true);
    mtc::Stage* contact_ptr = contact.get();
    task.add(std::move(contact));
    {
      auto connect = std::make_unique<mtc::stages::Connect>(
        "Pre-Grasp (OMPL)", mtc::stages::Connect::GroupPlannerVector{{arm, ompl}});
      connect->setTimeout(profile.timeout);
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
      approach->setMinMaxDistance(profile.approach_distance > 0. ? profile.approach_distance : config_.approach_min,
                                  profile.approach_distance > 0. ? profile.approach_distance : config_.approach_max);
      geometry_msgs::msg::Vector3Stamped down;
      auto grasp_direction = config_.approach_direction;
      const double cs = std::cos(profile.grasp_yaw), sn = std::sin(profile.grasp_yaw);
      grasp_direction[0] = cs*config_.approach_direction[0]-sn*config_.approach_direction[1];
      grasp_direction[1] = sn*config_.approach_direction[0]+cs*config_.approach_direction[1];
      down = direction(grasp_direction);
      approach->setDirection(down);
      pick->insert(std::move(approach));

      auto grasp = std::make_unique<mtc::stages::GeneratePose>("generate grasp poses");
      grasp->properties().configureInitFrom(mtc::Stage::PARENT);
      geometry_msgs::msg::PoseStamped grasp_target;
      grasp_target.header.frame_id = object;
      grasp_target.pose.orientation.z = std::sin(profile.grasp_yaw/2.);
      grasp_target.pose.orientation.w = std::cos(profile.grasp_yaw/2.);
      grasp->setPose(grasp_target);
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
         Eigen::AngleAxisd(config_.grasp_pitch, Eigen::Vector3d::UnitY())).toRotationMatrix();
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
      close->setGoal(std::map<std::string, double>{{config_.hand_joint, grip_target}});
      pick->insert(std::move(close));
      auto attach = std::make_unique<mtc::stages::ModifyPlanningScene>("Attach in PlanningScene");
      attach->attachObject(object, tool);
      attached = attach.get();
      pick->insert(std::move(attach));
      auto lift = std::make_unique<mtc::stages::MoveRelative>("Lift (Cartesian)", cartesian);
      lift->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      lift->setIKFrame(tool);
      lift->setMinMaxDistance(config_.lift_min, config_.lift_max);
      geometry_msgs::msg::Vector3Stamped up;
      up = direction(config_.lift_direction);
      lift->setDirection(up);
      pick->insert(std::move(lift));
      task.add(std::move(pick));
    }
    if (!profile.waypoint.empty()) {
      auto waypoint = std::make_unique<mtc::stages::MoveTo>("Transport waypoint (OMPL)", transport_ompl);
      waypoint->setGroup(arm); waypoint->setIKFrame(tool); waypoint->setTimeout(profile.timeout);
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = config_.motion_frame;
      pose.pose.position.x = profile.waypoint[0]; pose.pose.position.y = profile.waypoint[1]; pose.pose.position.z = profile.waypoint[2];
      pose.pose.orientation.x = profile.waypoint[3]; pose.pose.orientation.y = profile.waypoint[4];
      pose.pose.orientation.z = profile.waypoint[5]; pose.pose.orientation.w = profile.waypoint[6];
      waypoint->setGoal(pose); task.add(std::move(waypoint));
    }
    {
      auto connect = std::make_unique<mtc::stages::Connect>(
        "Transport (OMPL)", mtc::stages::Connect::GroupPlannerVector{{arm, transport_ompl}});
      connect->setTimeout(transport_keep_orientation ? config_.constrained_connect_timeout_sec : profile.timeout);
      if (transport_keep_orientation) {
        // The attached-object collision model alone does not keep a friction
        // grasp upright. Bound all rotation-vector axes relative to release
        // so OMPL cannot insert a redundant wrist turn during translation.
        const auto& q = place_pose.pose.orientation;
        Eigen::Quaterniond object_orientation(q.w, q.x, q.y, q.z);
        if (!object_orientation.coeffs().allFinite() || object_orientation.norm() < 1e-6) {
          throw std::invalid_argument("Invalid place orientation for transport constraint");
        }
        const Eigen::Quaterniond tool_orientation = object_orientation.normalized() * Eigen::Quaterniond(Eigen::AngleAxisd(profile.grasp_yaw, Eigen::Vector3d::UnitZ())) *
          Eigen::Quaterniond(Eigen::AngleAxisd(-config_.grasp_pitch, Eigen::Vector3d::UnitY()) *
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
      down = direction(profile.place_direction);
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
      open->setGoal(std::map<std::string, double>{{config_.hand_joint, open_target}});
      place->insert(std::move(open));
      auto detach = std::make_unique<mtc::stages::ModifyPlanningScene>("Detach in PlanningScene");
      detach->detachObject(object, tool);
      place->insert(std::move(detach));
      auto retreat = std::make_unique<mtc::stages::MoveRelative>("Retreat (Cartesian)", cartesian);
      retreat->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      retreat->setIKFrame(tool);
      retreat->setMinMaxDistance(config_.retreat_min, config_.retreat_max);
      geometry_msgs::msg::Vector3Stamped up;
      up = direction(profile.retreat_direction);
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
      home->setGoal(config_.home_state);
      task.add(std::move(home));
    }
    return task;
  }

  rclcpp::Node::SharedPtr node_;
  pick_place_mtc::TaskConfig config_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  std::mutex feedback_mutex_;
  std::optional<sensor_msgs::msg::JointState> feedback_;
  std::chrono::steady_clock::time_point feedback_arrived_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr object_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr place_sub_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr scene_client_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr plan_ready_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metrics_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr candidate_revision_pub_;
  rclcpp::Publisher<moveit_task_constructor_msgs::msg::Solution>::SharedPtr solution_pub_;
  rclcpp::Publisher<moveit_task_constructor_msgs::msg::TaskDescription>::SharedPtr description_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr scene_revision_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr scene_ready_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_guard_;
  std::atomic<bool> planning_active_{true};
  std::atomic<uint64_t> scene_revision_{0};
  std::atomic<bool> scene_revision_seen_{false};
  std::atomic<bool> scene_ready_{false};
  std::mutex poses_mutex_;
  std::optional<geometry_msgs::msg::PoseStamped> object_pose_;
  std::optional<geometry_msgs::msg::PoseStamped> place_pose_;
  moveit::core::RobotModelConstPtr model_;
  mtc::Task task_;
  std::vector<std::unique_ptr<mtc::Task>> tasks_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  std::thread spin_thread;
  std::exception_ptr spin_error;
  bool success = false;
  try {
    auto planner = std::make_shared<PickPlacePlanner>();
    executor.add_node(planner->node_base());
    spin_thread = std::thread([&] {
      try { executor.spin(); }
      catch (...) { spin_error = std::current_exception(); rclcpp::shutdown(); }
    });
    try { success = planner->run(); }
    catch (...) { executor.cancel(); spin_thread.join(); throw; }
    if (!success) executor.cancel();
    // Successful planning retains introspection until interrupted.
    spin_thread.join();
    if (spin_error) std::rethrow_exception(spin_error);
  } catch (const std::exception& error) {
    RCLCPP_ERROR(rclcpp::get_logger("plan_pick_place"), "%s", error.what());
    success = false;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return success ? 0 : 1;
}
