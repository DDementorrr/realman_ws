#include <iostream>
#include <moveit/task_constructor/task.h>
#include <moveit/robot_state/conversions.h>
#include <pick_place_mtc/candidate_audit.hpp>

using pick_place_mtc::CandidateAudit;
void require(bool ok, const std::string& label) {
  if (!ok) throw std::runtime_error(label);
  std::cout << "PASS " << label << std::endl;
}
template<class Message> Message decode(const std::string& data) {
  rclcpp::SerializedMessage bytes(data.size()/2);
  auto& buffer = bytes.get_rcl_serialized_message();
  buffer.buffer_length = data.size()/2;
  for (size_t i=0; i<buffer.buffer_length; ++i)
    buffer.buffer[i] = static_cast<unsigned char>(std::stoul(data.substr(2*i,2),nullptr,16));
  Message value; rclcpp::Serialization<Message>().deserialize_message(&bytes,&value); return value;
}
int main(int argc, char** argv) {
  rclcpp::init(argc,argv);
  try {
    {
    auto node = std::make_shared<rclcpp::Node>("test_candidate_audit",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    const auto config = pick_place_mtc::TaskConfig::load(*node);
    moveit::task_constructor::Task model_task; model_task.loadRobotModel(node);
    config.validate_model(*model_task.getRobotModel());
    const auto report = YAML::LoadFile(node->get_parameter("audit_archive").as_string());
    auto scene = std::make_shared<planning_scene::PlanningScene>(model_task.getRobotModel());
    require(scene->setPlanningSceneMsg(decode<moveit_msgs::msg::PlanningScene>(report["binding"]["scene_cdr_hex"].as<std::string>())), "actual archived initial scene imports");
    const auto initial = scene->getCurrentState();
    CandidateAudit valid(config,.001,.01); valid.state(scene,initial);
    require(valid.rejection.empty(),"actual initial robot state passes full collision/limit checks");
    auto q = initial; q.setVariablePosition("joint1",100.);
    CandidateAudit position(config,.001,.01); position.state(scene,q);
    require(position.rejection=="joint_position_limit","out-of-range commanded position rejected");
    q = initial; q.setVariableVelocity("joint1",100.);
    CandidateAudit velocity(config,.001,.01); velocity.state(scene,q);
    require(velocity.rejection.find("velocity_limit")!=std::string::npos,"excess commanded velocity rejected");
    q = initial; q.setVariableAcceleration("joint1",100.);
    CandidateAudit acceleration(config,.001,.01); acceleration.state(scene,q);
    require(acceleration.rejection.find("acceleration_limit")!=std::string::npos,"excess commanded acceleration rejected");
    CandidateAudit margin(config,10.,.01); margin.state(scene,initial);
    require(margin.rejection=="minimum_world_clearance","collision-free state below requested clearance rejected");
    const auto* camera = model_task.getRobotModel()->getLinkModel("wrist_camera_link");
    require(camera && !camera->getShapes().empty(),"mounted camera participates in the real collision model");
    moveit_msgs::msg::CollisionObject obstacle;
    obstacle.id="audit_camera_contact"; obstacle.header.frame_id=model_task.getRobotModel()->getModelFrame();
    shape_msgs::msg::SolidPrimitive box; box.type=box.BOX; box.dimensions={.002,.002,.002};
    obstacle.primitives={box};
    const auto p=initial.getGlobalLinkTransform("wrist_camera_link").translation();
    geometry_msgs::msg::Pose pose; pose.position.x=p.x(); pose.position.y=p.y(); pose.position.z=p.z(); pose.orientation.w=1.;
    obstacle.primitive_poses={pose}; obstacle.operation=obstacle.ADD;
    require(scene->processCollisionObjectMsg(obstacle),"camera-only test obstacle applies");
    collision_detection::CollisionRequest request; request.contacts=true; request.max_contacts=100;
    collision_detection::CollisionResult result; scene->checkCollision(request,result,initial);
    bool camera_contact=false;
    for (const auto& entry: result.contacts)
      camera_contact |= entry.first.first=="wrist_camera_link" || entry.first.second=="wrist_camera_link";
    require(camera_contact,"FCL explicitly reports camera/obstacle contact");
    CandidateAudit collision(config,.001,.01); collision.state(scene,initial);
    require(collision.rejection=="collision","camera contact cannot be offset by any soft score");
    }
    rclcpp::shutdown(); return 0;
  } catch(const std::exception& error) {
    std::cerr << "FAIL " << error.what() << std::endl;
    rclcpp::shutdown(); return 1;
  }
}
