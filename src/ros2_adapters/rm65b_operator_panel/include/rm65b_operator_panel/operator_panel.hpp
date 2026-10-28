#pragma once
#include <rviz_common/panel.hpp>
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_frame.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <QPointer>
#include <QTimer>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QTableWidget>
#include <QPushButton>
#include <QComboBox>
#include <chrono>

namespace rm65b_operator_panel {
class OperatorPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit OperatorPanel(QWidget* parent = nullptr);
  void onInitialize() override;
  ~OperatorPanel() override;
private:
  bool eventFilter(QObject*, QEvent*) override;
  void tick();
  bool bindDisplay();
  void initializeModel();
  void syncGoal();
  void invalidate();
  void applyPose(const geometry_msgs::msg::PoseStamped&);
  void applyJoints();
  void applyLimits();
  void receiveObject(const geometry_msgs::msg::PoseStamped&, bool truth);
  void notice(const QString&);
  bool freshJoints() const;
  geometry_msgs::msg::PoseStamped stamped(const Eigen::Isometry3d&, const std::string&) const;
  Eigen::Isometry3d inBase(const geometry_msgs::msg::PoseStamped&);
  void displayPose(int row, const Eigen::Isometry3d&);
  void publishLimits();
  void guardNativeControls();
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::unique_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr input_sub_, object_sub_, truth_sub_;
  rclcpp::Subscription<visualization_msgs::msg::InteractiveMarkerFeedback>::SharedPtr marker_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pub_, goal_pub_, requested_pub_, object_base_pub_, object_tcp_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr goal_joints_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_, limits_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr task_client_;
  QPointer<moveit_rviz_plugin::MotionPlanningDisplay> display_;
  QPointer<moveit_rviz_plugin::MotionPlanningFrame> native_;
  QPointer<QDoubleSpinBox> native_scale_;
  QPointer<QPushButton> execute_, plan_, plan_execute_;
  QTimer timer_;
  QTableWidget *poses_, *joints_;
  QLabel *status_, *connection_, *limits_label_;
  QDoubleSpinBox *tcp_limit_, *requested_scale_;
  QComboBox* object_source_;
  QPushButton *pose_apply_, *joint_apply_, *limits_apply_, *task_execute_;
  std::vector<QDoubleSpinBox*> pose_edits_, joint_edits_, limit_edits_;
  std::vector<std::string> variables_;
  std::vector<double> vmax_, radii_, last_goal_;
  moveit::core::RobotModelConstPtr model_;
  std::unique_ptr<moveit::core::RobotState> actual_;
  const moveit::core::JointModelGroup* group_ = nullptr;
  std::string base_ = "base_link", tcp_, group_name_;
  sensor_msgs::msg::JointState last_joints_;
  geometry_msgs::msg::PoseStamped object_;
  std::chrono::steady_clock::time_point joints_received_{}, object_received_{};
  bool initialized_ = false, received_joints_ = false, received_object_ = false;
  bool limits_dirty_ = false, goal_valid_ = true, limits_supported_ = false;
  bool plan_valid_ = false, syncing_ = false;
  double cap_ = 1.0;
  std::uint64_t revision_ = 0, planning_revision_ = 0;
  int ticks_ = 0;
};
}
