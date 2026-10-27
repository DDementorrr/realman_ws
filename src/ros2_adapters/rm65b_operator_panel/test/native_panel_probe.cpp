// Integration fixture: loads the real RViz/MoveIt widgets against a simulation.
// It only runs when explicitly invoked; no test-command subscriber is shipped.
#include <rviz_common/visualization_frame.hpp>
#include <rviz_rendering/render_window.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/ros_integration/ros_client_abstraction.hpp>
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/srv/get_interactive_markers.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <moveit/robot_interaction/robot_interaction.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <QApplication>
#include <QTimer>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QTableWidget>
#include <QLabel>
#include <QTest>
#include <QScreen>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QFile>
#include <iostream>
#include <chrono>
#include <execinfo.h>
#include <signal.h>
#include <unistd.h>

int main(int argc, char** argv) {
  signal(SIGSEGV, +[](int){void* trace[40];int n=backtrace(trace,40);backtrace_symbols_fd(trace,n,2);_exit(139);});
  QApplication::setAttribute(Qt::AA_UseDesktopOpenGL);
  QApplication app(argc, argv);
  if (argc < 3) { std::cerr << "native_panel_probe CONFIG OUTPUT_PREFIX [--ros-args ...]\n"; return 2; }
  const QString config = argv[1], output = argv[2];
  rviz_common::ros_integration::RosClientAbstraction ros;
  auto abstraction = ros.init(argc, argv, "rviz_panel_probe", false);
  auto frame = std::make_unique<rviz_common::VisualizationFrame>(abstraction);
  frame->setApp(&app); frame->setSplashPath(""); frame->initialize(abstraction, config); frame->resize(1900, 1000); frame->show();
  auto node = std::make_shared<rclcpp::Node>("panel_probe_observer", rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)}));
  rclcpp::executors::SingleThreadedExecutor exec; exec.add_node(node);
  geometry_msgs::msg::PoseStamped current, accepted, object_base, object_tcp;
  sensor_msgs::msg::JointState actual;
  moveit_msgs::msg::DisplayTrajectory trajectory;
  QJsonObject limits, result; QJsonArray passed;
  const auto qos = rclcpp::QoS(1).transient_local().reliable();
  auto sub_current = node->create_subscription<geometry_msgs::msg::PoseStamped>("/manipulation/rm65b/current_pose", qos, [&](geometry_msgs::msg::PoseStamped::ConstSharedPtr p){current=*p;});
  auto sub_goal = node->create_subscription<geometry_msgs::msg::PoseStamped>("/manipulation/rm65b/goal_pose", qos, [&](geometry_msgs::msg::PoseStamped::ConstSharedPtr p){accepted=*p;});
  auto sub_object_base = node->create_subscription<geometry_msgs::msg::PoseStamped>("/manipulation/rm65b/object_in_base", qos, [&](geometry_msgs::msg::PoseStamped::ConstSharedPtr p){object_base=*p;});
  auto sub_object_tcp = node->create_subscription<geometry_msgs::msg::PoseStamped>("/manipulation/rm65b/object_in_tcp", qos, [&](geometry_msgs::msg::PoseStamped::ConstSharedPtr p){object_tcp=*p;});
  auto sub_joints = node->create_subscription<sensor_msgs::msg::JointState>("/joint_states", rclcpp::SensorDataQoS(), [&](sensor_msgs::msg::JointState::ConstSharedPtr p){actual=*p;});
  auto sub_traj = node->create_subscription<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 10, [&](moveit_msgs::msg::DisplayTrajectory::ConstSharedPtr p){trajectory=*p;});
  auto sub_limits = node->create_subscription<std_msgs::msg::String>("/manipulation/rm65b/planning_limits", qos, [&](std_msgs::msg::String::ConstSharedPtr p){limits=QJsonDocument::fromJson(QByteArray::fromStdString(p->data)).object();});
  auto input = node->create_publisher<geometry_msgs::msg::PoseStamped>("/manipulation/rm65b/set_goal_pose", 10);
  QPointer<QPushButton> plan, execute; QPointer<QWidget> panel;
  QPointer<moveit_rviz_plugin::MotionPlanningDisplay> display;
  rclcpp::Publisher<visualization_msgs::msg::InteractiveMarkerFeedback>::SharedPtr feedback;
  rclcpp::Client<visualization_msgs::srv::GetInteractiveMarkers>::SharedPtr markers;
  geometry_msgs::msg::PoseStamped before;
  double initial_j1=0, target_j1=0; int stage=0, stage_ticks=0, total=0;
  bool marker_requested=false, marker_response=false; std::string marker_name;
  auto click = [&](const char* name) { auto* w=panel->findChild<QPushButton*>(name); if(!w) throw std::runtime_error(name); QTest::mouseClick(w, Qt::LeftButton); };
  auto edit = [&](const char* name)->QDoubleSpinBox* {auto* w=panel->findChild<QDoubleSpinBox*>(name); if(!w) throw std::runtime_error(name);return w;};
  auto passed_test=[&](const char* name){passed.append(name);std::cout<<"PASS "<<name<<std::endl;};
  auto finish=[&](bool ok, const QString& reason){
    result["accepted"]=ok;result["reason"]=reason;result["checks"]=passed;result["limits"]=limits;
    QFile f(output+".json");f.open(QIODevice::WriteOnly);f.write(QJsonDocument(result).toJson());f.close();
    frame->grab().save(output+".png");
    frame->getRenderWindow()->captureScreenShot((output+"-scene.png").toStdString());std::cout<<(ok?"ACCEPTED ":"FAILED ")<<reason.toStdString()<<std::endl;
    app.exit(ok?0:1);
  };
  QTimer timer;
  QObject::connect(&timer, &QTimer::timeout, [&]{
    exec.spin_some(std::chrono::milliseconds(2)); ++total; ++stage_ticks;
    try {
      if(total>1000) { finish(false,QString("timeout stage %1").arg(stage));timer.stop();return; }
      if(!panel) {
        for(auto* w:QApplication::allWidgets()) {
          if(w->metaObject()->className()==QString("rm65b_operator_panel::OperatorPanel")) panel=w;
          if(auto* button=qobject_cast<QPushButton*>(w)) {if(button->objectName()=="plan_button")plan=button;if(button->objectName()=="execute_button")execute=button;}
        }
        auto* root=frame->getManager()->getRootDisplayGroup();
        for(int i=0;i<root->numDisplays();++i) if(auto* d=qobject_cast<moveit_rviz_plugin::MotionPlanningDisplay*>(root->getDisplayAt(i)))display=d;
      }
      if(!panel||!plan||!execute||!display||current.header.frame_id.empty()||!display->getRobotModel()||limits.isEmpty()||actual.position.empty())return;
      switch(stage) {
      case 0:
        initial_j1=actual.position.at(std::distance(actual.name.begin(), std::find(actual.name.begin(), actual.name.end(), "joint1"))); click("copy_current");
        edit("joint_edit_0")->setValue(initial_j1+0.04);click("apply_joints");target_j1=edit("joint_edit_0")->value();
        passed_test("plugin loaded and actual feedback/FK available");stage=1;stage_ticks=0;break;
      case 1:
        if(stage_ticks<8)break;
        if(std::abs(display->getQueryGoalState()->getVariablePosition("joint1")-target_j1)>1e-4)throw std::runtime_error("native goal did not follow joint editor");
        if(std::abs(actual.position.at(std::distance(actual.name.begin(), std::find(actual.name.begin(), actual.name.end(), "joint1")))-initial_j1)>0.002)throw std::runtime_error("editing caused motion");
        passed_test("joint edit synchronizes native goal without execution");
        input->publish(accepted); before=accepted;stage=2;stage_ticks=0;break;
      case 2:
        if(stage_ticks<10)break;
        if(std::abs(accepted.pose.position.x-before.pose.position.x)>0.002)throw std::runtime_error("preview topic IK mismatch");
        passed_test("terminal preview pose performs IK without motion");
        edit("tcp_edit_0")->setValue(4);click("apply_tcp");stage=3;stage_ticks=0;break;
      case 3:
        if(stage_ticks<5)break;
        if(std::abs(accepted.pose.position.x-before.pose.position.x)>0.002)throw std::runtime_error("failed IK replaced accepted target");
        passed_test("unreachable IK refused and prior goal retained");
        click("copy_current");edit("joint_edit_0")->setValue(initial_j1+0.04);click("apply_joints");
        edit("joint_limit_0")->setValue(0.12);edit("tcp_speed_limit")->setValue(0.1);click("apply_limits");
        trajectory.trajectory.clear();QTest::mouseClick(plan,Qt::LeftButton);stage=4;stage_ticks=0;break;
      case 4:
        if(!execute->isEnabled()||trajectory.trajectory.empty())break;
        if(trajectory.trajectory[0].joint_trajectory.points.size()<2)throw std::runtime_error("empty native plan");
        {
          const auto effective=limits["effective_rad_s"].toArray(),radii=limits["jacobian_radius_bounds_m"].toArray();
          const auto names=limits["joint_names"].toArray();const auto& jt=trajectory.trajectory[0].joint_trajectory;
          double bound=0;
          for(const auto& p:jt.points) {
            if(p.velocities.size()!=jt.joint_names.size())throw std::runtime_error("trajectory missing velocities");
            double tcp=0;
            for(size_t i=0;i<jt.joint_names.size();++i)for(int k=0;k<names.size();++k)if(names[k].toString().toStdString()==jt.joint_names[i]){
              if(std::abs(p.velocities[i])>effective[k].toDouble()*1.01+1e-6)throw std::runtime_error("planned joint velocity exceeds panel cap");
              tcp+=radii[k].toDouble()*std::abs(p.velocities[i]);
            }
            bound=std::max(bound,tcp);
          }
          if(bound>0.101)throw std::runtime_error("TCP conservative speed cap exceeded");result["max_tcp_bound_m_s"]=bound;
        }
        passed_test("native Plan trajectory obeys joint and TCP conservative velocity caps");
        edit("joint_limit_0")->setValue(0.1);click("apply_limits");
        if(execute->isEnabled())throw std::runtime_error("old plan still executable after limits changed");
        passed_test("limit changes invalidate native Execute");
        trajectory.trajectory.clear();QTest::mouseClick(plan,Qt::LeftButton);stage=5;stage_ticks=0;break;
      case 5:
        if(stage_ticks<15||!execute->isEnabled()||trajectory.trajectory.empty())break;
        std::cout << "EXECUTING native q1=" << display->getQueryGoalState()->getVariablePosition("joint1") << " intended=" << target_j1 << std::endl;
        QTest::mouseClick(execute,Qt::LeftButton);stage=6;stage_ticks=0;break;
      case 6:
        if(stage_ticks%30==0) std::cout << "WAIT actual q1=" << actual.position.at(std::distance(actual.name.begin(), std::find(actual.name.begin(), actual.name.end(), "joint1"))) << " target=" << target_j1 << " " << panel->findChild<QLabel*>("operator_status")->text().toStdString() << std::endl;
        if(stage_ticks<15||std::abs(actual.position.at(std::distance(actual.name.begin(), std::find(actual.name.begin(), actual.name.end(), "joint1")))-target_j1)>0.003)break;
        passed_test("native Execute moves Gazebo actual joints to accepted target");
        if(object_base.header.frame_id!="base_link"||object_tcp.header.frame_id!="ctm2f110_tool0")throw std::runtime_error("object frame telemetry missing");
        passed_test("object base and TCP telemetry available");
        {
          const auto topic=display->getRobotInteraction()->getServerTopic();
          feedback=node->create_publisher<visualization_msgs::msg::InteractiveMarkerFeedback>(topic+"/feedback",10);
          markers=node->create_client<visualization_msgs::srv::GetInteractiveMarkers>(topic+"/get_interactive_markers");
        }
        before=accepted;stage=7;stage_ticks=0;break;
      case 7:
        if(!marker_requested&&markers->service_is_ready()) {
          marker_requested=true;
          markers->async_send_request(std::make_shared<visualization_msgs::srv::GetInteractiveMarkers::Request>(),[&](rclcpp::Client<visualization_msgs::srv::GetInteractiveMarkers>::SharedFuture f){
            for(const auto& m:f.get()->markers)if(m.name.find("goal")!=std::string::npos){
              visualization_msgs::msg::InteractiveMarkerFeedback p;p.header=m.header;p.client_id="panel_integration_probe";p.marker_name=m.name;
              p.event_type=p.POSE_UPDATE;p.pose=m.pose;p.pose.position.x+=0.003;feedback->publish(p);marker_name=m.name;marker_response=true;break;
            }
          });
        }
        if(!marker_response||stage_ticks<15)break;
        if(std::abs(accepted.pose.position.x-before.pose.position.x)<0.001)break;
        if(std::abs(actual.position.at(std::distance(actual.name.begin(), std::find(actual.name.begin(), actual.name.end(), "joint1")))-target_j1)>0.004)throw std::runtime_error("marker preview caused execution");
        result["marker_name"]=QString::fromStdString(marker_name);passed_test("native marker feedback synchronizes accepted goal without motion");
        finish(true,"RViz native-widget + Gazebo integration passed");timer.stop();break;
      }
    }catch(const std::exception& e){finish(false,e.what());timer.stop();}
  });
  timer.start(100);const int rc=app.exec();timer.stop();exec.remove_node(node);frame.reset();ros.shutdown();return rc;
}
