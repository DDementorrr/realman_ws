#include "rm65b_operator_panel/operator_panel.hpp"
#include <rviz_common/display_context.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <moveit/robot_interaction/robot_interaction.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <QApplication>
#include <QBoxLayout>
#include <QHeaderView>
#include <QEvent>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QSignalBlocker>
#include <QDockWidget>
#include <QMainWindow>
#include <QFontDatabase>
#include <QFontMetrics>
#include <QDebug>
#include <cmath>
#include <algorithm>

namespace rm65b_operator_panel {
namespace {
constexpr const char* PREFIX = "/manipulation/rm65b/";
QDoubleSpinBox* spin(QWidget* parent, double lo, double hi, int decimals = 5) {
  auto* w = new QDoubleSpinBox(parent); w->setRange(lo, hi); w->setDecimals(decimals);
  w->setSingleStep(0.01); w->setKeyboardTracking(false); return w;
}
std::array<double, 6> xyzrpy(const Eigen::Isometry3d& t) {
  const auto q = Eigen::Quaterniond(t.rotation());
  tf2::Quaternion qt(q.x(), q.y(), q.z(), q.w()); double r, p, y;
  tf2::Matrix3x3(qt).getRPY(r, p, y);
  return {t.translation().x(), t.translation().y(), t.translation().z(), r, p, y};
}
moveit_rviz_plugin::MotionPlanningDisplay* findDisplay(rviz_common::DisplayGroup* group) {
  for (int i = 0; i < group->numDisplays(); ++i) {
    auto* d = group->getDisplayAt(i);
    if (auto* planning = qobject_cast<moveit_rviz_plugin::MotionPlanningDisplay*>(d)) return planning;
    if (auto* nested = qobject_cast<rviz_common::DisplayGroup*>(d))
      if (auto* found = findDisplay(nested)) return found;
  }
  return nullptr;
}
}
OperatorPanel::OperatorPanel(QWidget* parent) : Panel(parent) {
  // Select a real CJK font for this panel; never change RViz's global font.
  const QFontDatabase fonts;
  bool cjk_font = false;
  for (const auto& family : fonts.families(QFontDatabase::SimplifiedChinese)) {
    QFont candidate(font()); candidate.setFamily(family);
    if (!QFontMetrics(candidate).inFont(QChar(0x5173))) continue;  // 关
    setFont(candidate); cjk_font = true; break;
  }
  if (!cjk_font) qWarning() << "RM65B panel: CJK font missing; run scripts/setup_operator_panel_fonts.sh and restart RViz.";
  auto* layout = new QVBoxLayout(this);
  connection_ = new QLabel("等待 MoveIt", this);
  connection_->setWordWrap(true); layout->addWidget(connection_);
  poses_ = new QTableWidget(5, 6, this); poses_->setObjectName("tcp_table");
  poses_->setHorizontalHeaderLabels({"X m", "Y m", "Z m", "Roll rad", "Pitch rad", "Yaw rad"});
  poses_->setVerticalHeaderLabels({"当前 TCP", "目标 TCP", "物体/基座", "物体/TCP", "TCP 编辑"});
  for (int r = 0; r < 4; ++r) for (int c = 0; c < 6; ++c) {
    auto* item = new QTableWidgetItem("--"); item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
    poses_->setItem(r, c, item);
  }
  for (int c = 0; c < 6; ++c) {
    auto* w = spin(this, c < 3 ? -5.0 : -M_PI, c < 3 ? 5.0 : M_PI);
    w->setObjectName(QString("tcp_edit_%1").arg(c)); pose_edits_.push_back(w); poses_->setCellWidget(4, c, w);
  }
  poses_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  poses_->setMinimumHeight(185); poses_->setMaximumHeight(205); layout->addWidget(poses_);
  auto* buttons = new QHBoxLayout;
  pose_apply_ = new QPushButton("应用 TCP", this); pose_apply_->setObjectName("apply_tcp");
  auto* copy = new QPushButton("读取当前", this); copy->setObjectName("copy_current");
  buttons->addWidget(pose_apply_); buttons->addWidget(copy); layout->addLayout(buttons);
  joints_ = new QTableWidget(0, 6, this); joints_->setObjectName("joint_table");
  joints_->setHorizontalHeaderLabels({"关节", "当前 rad", "目标 rad", "模型上限\nrad/s", "请求上限\nrad/s", "生效上限\nrad/s"});
  joints_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
  joints_->setMinimumHeight(220); layout->addWidget(joints_);
  joint_apply_ = new QPushButton("应用关节", this); joint_apply_->setObjectName("apply_joints");
  layout->addWidget(joint_apply_);
  auto* speed = new QHBoxLayout; speed->addWidget(new QLabel("TCP 上限 m/s", this));
  tcp_limit_ = spin(this, 0, 2, 4); tcp_limit_->setObjectName("tcp_speed_limit"); tcp_limit_->setValue(0.1);
  tcp_limit_->setSingleStep(0.05); tcp_limit_->setSpecialValueText("不限"); speed->addWidget(tcp_limit_);
  speed->addWidget(new QLabel("规划倍率", this));
  requested_scale_ = spin(this, 0.000001, 1, 6); requested_scale_->setObjectName("requested_velocity_scaling");
  requested_scale_->setSingleStep(0.05); requested_scale_->setValue(0.1); speed->addWidget(requested_scale_);
  layout->addLayout(speed);
  limits_apply_ = new QPushButton("应用限速", this); limits_apply_->setObjectName("apply_limits"); layout->addWidget(limits_apply_);
  limits_label_ = new QLabel("限速：--", this); limits_label_->setWordWrap(true); layout->addWidget(limits_label_);
  object_source_ = new QComboBox(this); object_source_->setObjectName("object_source");
  object_source_->addItems({"/object_pose", "/pick_place/sim/object_pose [sim]"});
  layout->addWidget(object_source_);
  task_execute_ = new QPushButton("执行抓放", this); task_execute_->setObjectName("execute_pick_place");
  layout->addWidget(task_execute_);
  status_ = new QLabel("未连接", this); status_->setObjectName("operator_status"); status_->setWordWrap(true); layout->addWidget(status_);
  connect(pose_apply_, &QPushButton::clicked, this, [this] {
    if (!actual_) return;
    tf2::Quaternion q; q.setRPY(pose_edits_[3]->value(), pose_edits_[4]->value(), pose_edits_[5]->value());
    geometry_msgs::msg::PoseStamped p; p.header.frame_id = base_; p.header.stamp = node_->now();
    p.pose.position.x = pose_edits_[0]->value(); p.pose.position.y = pose_edits_[1]->value(); p.pose.position.z = pose_edits_[2]->value();
    p.pose.orientation.x = q.x(); p.pose.orientation.y = q.y(); p.pose.orientation.z = q.z(); p.pose.orientation.w = q.w(); applyPose(p);
  });
  connect(copy, &QPushButton::clicked, this, [this] {
    if (!actual_ || !freshJoints()) return;
    const auto values = xyzrpy(actual_->getGlobalLinkTransform(base_).inverse() * actual_->getGlobalLinkTransform(tcp_));
    for (size_t i = 0; i < 6; ++i) pose_edits_[i]->setValue(values[i]);
    for (size_t i = 0; i < variables_.size(); ++i) joint_edits_[i]->setValue(actual_->getVariablePosition(variables_[i]));
  });
  connect(joint_apply_, &QPushButton::clicked, this, &OperatorPanel::applyJoints);
  connect(limits_apply_, &QPushButton::clicked, this, &OperatorPanel::applyLimits);
  connect(tcp_limit_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this] { limits_dirty_ = true; invalidate(); });
  connect(requested_scale_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this] { limits_dirty_ = true; invalidate(); });
  connect(object_source_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this] { received_object_ = false; });
  connect(task_execute_, &QPushButton::clicked, this, [this] {
    if (!task_client_ || !task_client_->service_is_ready()) { notice("Pick & Place 服务未就绪"); return; }
    task_execute_->setEnabled(false);
    task_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(), [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f) {
      task_execute_->setEnabled(true); const auto r = f.get(); notice(QString(r->success ? "任务已接受：" : "任务拒绝：") + QString::fromStdString(r->message));
    });
  });
  connect(&timer_, &QTimer::timeout, this, &OperatorPanel::tick);
}
OperatorPanel::~OperatorPanel() { timer_.stop(); if (node_) executor_.remove_node(node_); }
void OperatorPanel::onInitialize() {
  auto raw = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  const bool sim = raw->has_parameter("use_sim_time") && raw->get_parameter("use_sim_time").as_bool();
  node_ = std::make_shared<rclcpp::Node>("rm65b_operator_panel", rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", sim)}));
  executor_.add_node(node_);
  tf_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_, node_, false);
  auto qos = rclcpp::QoS(1).reliable().transient_local();
  auto publisher = [this, &qos](const std::string& suffix) { return node_->create_publisher<geometry_msgs::msg::PoseStamped>(std::string(PREFIX) + suffix, qos); };
  current_pub_ = publisher("current_pose"); goal_pub_ = publisher("goal_pose"); requested_pub_ = publisher("ik_target_pose");
  object_base_pub_ = publisher("object_in_base"); object_tcp_pub_ = publisher("object_in_tcp");
  goal_joints_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(std::string(PREFIX) + "goal_joints", qos);
  status_pub_ = node_->create_publisher<std_msgs::msg::String>(std::string(PREFIX) + "panel_status", qos);
  limits_pub_ = node_->create_publisher<std_msgs::msg::String>(std::string(PREFIX) + "planning_limits", qos);
  joints_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>("/joint_states", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::JointState::ConstSharedPtr p) {
    last_joints_ = *p; joints_received_ = std::chrono::steady_clock::now(); received_joints_ = true;
  });
  input_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(std::string(PREFIX) + "set_goal_pose", 10, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr p) { applyPose(*p); });
  object_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>("/object_pose", rclcpp::SensorDataQoS(), [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr p) { receiveObject(*p, false); });
  truth_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>("/pick_place/sim/object_pose", 10, [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr p) { receiveObject(*p, true); });
  task_client_ = node_->create_client<std_srvs::srv::Trigger>("/pick_place/execute");
  QTimer::singleShot(0, this, [this] {
    auto* dock = qobject_cast<QDockWidget*>(parentWidget());
    auto* window = dock ? qobject_cast<QMainWindow*>(dock->parentWidget()) : nullptr;
    if (window) {
      window->addDockWidget(Qt::RightDockWidgetArea, dock);
      dock->setMinimumWidth(620);
      window->resizeDocks({dock}, {650}, Qt::Horizontal);
    }
  });
  timer_.start(40);
}
bool OperatorPanel::bindDisplay() {
  if (display_ && native_ && initialized_ && display_->getRobotModel()) return true;
  display_ = findDisplay(getDisplayContext()->getRootDisplayGroup());
  if (!display_ || !display_->getRobotModel() || !display_->getRobotInteraction() ||
      !display_->getQueryGoalStateHandler() || !display_->getQueryStartStateHandler()) return false;
  for (auto* w : QApplication::allWidgets()) if (auto* f = qobject_cast<moveit_rviz_plugin::MotionPlanningFrame*>(w)) { native_ = f; break; }
  if (!native_) return false;
  native_scale_ = native_->findChild<QDoubleSpinBox*>("velocity_scaling_factor");
  execute_ = native_->findChild<QPushButton*>("execute_button");
  plan_ = native_->findChild<QPushButton*>("plan_button");
  plan_execute_ = native_->findChild<QPushButton*>("plan_and_execute_button");
  if (!native_scale_ || !execute_ || !plan_ || !plan_execute_) { notice("原生 MoveIt 控件版本不兼容；面板未接管目标"); return false; }
  native_scale_->setDecimals(6); native_scale_->setRange(0.000001, 1.0); native_scale_->setSingleStep(0.01);
  { QSignalBlocker block(requested_scale_); requested_scale_->setValue(native_scale_->value()); }
  for (auto* button : {execute_.data(), plan_.data(), plan_execute_.data()}) button->installEventFilter(this);
  connect(display_, &moveit_rviz_plugin::MotionPlanningDisplay::queryGoalStateChanged, this, &OperatorPanel::syncGoal, Qt::QueuedConnection);
  connect(native_, &moveit_rviz_plugin::MotionPlanningFrame::planningFinished, this, [this] {
    // The native worker enables Execute before this queued signal. Our timer
    // may already have disabled it; inspect the native result, then restore it.
    auto* result = native_->findChild<QLabel*>("result_label");
    const bool success = result && result->text().startsWith("Time:");
    plan_valid_ = success && planning_revision_ == revision_ && goal_valid_ && !limits_dirty_;
    execute_->setEnabled(plan_valid_ && freshJoints());
  }, Qt::QueuedConnection);
  connect(plan_, &QPushButton::clicked, this, [this] { planning_revision_ = revision_; plan_valid_ = false; execute_->setEnabled(false); });
  connect(native_scale_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double value) {
    if (syncing_) return;
    invalidate();
    { QSignalBlocker block(requested_scale_); requested_scale_->setValue(value); }
    if (initialized_ && value > cap_) { QSignalBlocker block(native_scale_); native_scale_->setValue(cap_); }
    if (initialized_) publishLimits();
  });
  const auto topic = display_->getRobotInteraction()->getServerTopic() + "/feedback";
  marker_sub_ = node_->create_subscription<visualization_msgs::msg::InteractiveMarkerFeedback>(topic, 10, [this](visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr p) {
    if (p->event_type != visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE || p->marker_name.find("goal") == std::string::npos) return;
    geometry_msgs::msg::PoseStamped requested; requested.header = p->header; requested.pose = p->pose;
    requested_pub_->publish(requested);
  });
  initializeModel(); return initialized_;
}
void OperatorPanel::initializeModel() {
  model_ = display_->getRobotModel(); group_name_ = display_->getCurrentPlanningGroup();
  group_ = model_->getJointModelGroup(group_name_);
  if (!group_ || group_name_ != "rm65_arm" || !model_->hasLinkModel(base_)) { notice("面板需要 rm65_arm 规划组与 base_link"); return; }
  tcp_ = model_->hasLinkModel("ctm2f110_tool0") ? "ctm2f110_tool0" : "Link6";
  actual_ = std::make_unique<moveit::core::RobotState>(model_); actual_->setToDefaultValues(); actual_->update();
  variables_ = group_->getVariableNames(); joints_->setRowCount(static_cast<int>(variables_.size()));
  limits_supported_ = true;
  for (size_t i = 0; i < variables_.size(); ++i) {
    const auto& b = model_->getVariableBounds(variables_[i]); vmax_.push_back(b.max_velocity_);
    if (!b.velocity_bounded_ || !std::isfinite(b.max_velocity_) || b.max_velocity_ <= 0) limits_supported_ = false;
    auto* position = spin(this, b.position_bounded_ ? b.min_position_ : -2*M_PI, b.position_bounded_ ? b.max_position_ : 2*M_PI);
    position->setObjectName(QString("joint_edit_%1").arg(i)); joint_edits_.push_back(position); joints_->setCellWidget(i, 2, position);
    auto* limit = spin(this, 0.00001, std::max(0.00001, b.max_velocity_), 6); limit->setValue(b.max_velocity_);
    limit->setObjectName(QString("joint_limit_%1").arg(i)); limit_edits_.push_back(limit); joints_->setCellWidget(i, 4, limit);
    connect(limit, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this] { limits_dirty_ = true; invalidate(); });
    for (int c : {0, 1, 3, 5}) {
      auto* item = new QTableWidgetItem(c == 0 ? QString::fromStdString(variables_[i]) : c == 3 ? QString::number(b.max_velocity_, 'f', 5) : "--");
      item->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable); joints_->setItem(i, c, item);
    }
  }
  // For a serial revolute chain, the sum of downstream origin lengths bounds each
  // translational Jacobian column for every configuration, including between waypoints.
  std::map<std::string, double> radii; double radius = 0;
  for (auto* link = model_->getLinkModel(tcp_); link && link->getParentJointModel(); link = link->getParentLinkModel()) {
    const auto* joint = link->getParentJointModel();
    if (joint->getType() == moveit::core::JointModel::REVOLUTE) radii[joint->getName()] = radius;
    else if (joint->getType() != moveit::core::JointModel::FIXED) limits_supported_ = false;
    radius += link->getJointOriginTransform().translation().norm();
  }
  for (const auto& name : variables_) {
    const auto* joint = model_->getJointOfVariable(name);
    if (!radii.count(joint->getName())) limits_supported_ = false;
    radii_.push_back(radii[joint->getName()]);
  }
  initialized_ = true;
  connection_->setText(QString("%1 | TCP %2 | %3").arg(QString::fromStdString(base_), QString::fromStdString(tcp_), QString::fromStdString(group_name_)));
  applyLimits(); syncGoal();
}
void OperatorPanel::invalidate() {
  ++revision_; plan_valid_ = false;
  if (execute_) execute_->setEnabled(false);
  if (display_ && initialized_) display_->dropVisualizedTrajectory();
}
void OperatorPanel::guardNativeControls() {
  if (execute_ && (!plan_valid_ || limits_dirty_ || !goal_valid_ || !freshJoints())) execute_->setEnabled(false);
}
bool OperatorPanel::eventFilter(QObject* obj, QEvent* event) {
  if (event->type() == QEvent::MouseButtonPress || event->type() == QEvent::KeyPress) {
    if (obj == execute_ || obj == plan_ || obj == plan_execute_) {
      if (!initialized_ || !freshJoints() || !goal_valid_ || limits_dirty_ || !limits_supported_ ||
          display_->getCurrentPlanningGroup() != group_name_ || (obj == execute_ && !plan_valid_)) {
        notice("未就绪或计划已过期"); return true;
      }
    }
  }
  return Panel::eventFilter(obj, event);
}
bool OperatorPanel::freshJoints() const {
  if (!received_joints_ || !initialized_) return false;
  if (std::chrono::duration<double>(std::chrono::steady_clock::now() - joints_received_).count() > 2.0) return false;
  const double age = (node_->now() - rclcpp::Time(last_joints_.header.stamp, node_->get_clock()->get_clock_type())).seconds();
  if (!std::isfinite(age) || age < -0.1 || age > 2.0) return false;
  for (const auto& name : variables_) {
    auto it = std::find(last_joints_.name.begin(), last_joints_.name.end(), name);
    const auto index = std::distance(last_joints_.name.begin(), it);
    if (it == last_joints_.name.end() || size_t(index) >= last_joints_.position.size() || !std::isfinite(last_joints_.position[index])) return false;
  }
  return true;
}
geometry_msgs::msg::PoseStamped OperatorPanel::stamped(const Eigen::Isometry3d& t, const std::string& frame) const {
  geometry_msgs::msg::PoseStamped p; p.header.frame_id = frame; p.header.stamp = node_->now(); p.pose = tf2::toMsg(t); return p;
}
Eigen::Isometry3d OperatorPanel::inBase(const geometry_msgs::msg::PoseStamped& p) {
  if (p.header.frame_id.empty()) throw std::runtime_error("frame_id 不能为空");
  const auto& a = p.pose; const double norm = std::sqrt(a.orientation.x*a.orientation.x + a.orientation.y*a.orientation.y + a.orientation.z*a.orientation.z + a.orientation.w*a.orientation.w);
  if (!std::isfinite(a.position.x) || !std::isfinite(a.position.y) || !std::isfinite(a.position.z) || !std::isfinite(norm) || norm < 1e-9) throw std::runtime_error("位姿包含非有限值或无效四元数");
  auto pose = a; pose.orientation.x /= norm; pose.orientation.y /= norm; pose.orientation.z /= norm; pose.orientation.w /= norm;
  Eigen::Isometry3d t; tf2::fromMsg(pose, t);
  if (p.header.frame_id != base_) t = tf2::transformToEigen(static_cast<tf2::BufferCore&>(*tf_).lookupTransform(base_, p.header.frame_id, tf2::TimePoint(std::chrono::nanoseconds(rclcpp::Time(p.header.stamp).nanoseconds())))) * t;
  return t;
}
void OperatorPanel::applyPose(const geometry_msgs::msg::PoseStamped& p) {
  requested_pub_->publish(p);
  if (!initialized_ || !freshJoints()) { notice("等待有效关节反馈"); return; }
  try {
    const auto target = actual_->getGlobalLinkTransform(base_) * inBase(p);
    moveit::core::RobotState candidate(*actual_);
    if (!candidate.setFromIK(group_, target, tcp_, 0.1)) throw std::runtime_error("IK 无解");
    candidate.update();
    if (!candidate.satisfiesBounds(group_)) throw std::runtime_error("IK 超出关节限位");
    if (display_->getPlanningSceneRO()->isStateColliding(candidate, group_name_)) throw std::runtime_error("IK 目标与场景碰撞");
    display_->setQueryGoalState(candidate); syncGoal(); notice("目标已更新");
  } catch (const std::exception& e) { invalidate(); notice(QString("目标拒绝：") + e.what()); }
}
void OperatorPanel::applyJoints() {
  if (!initialized_ || !freshJoints()) { notice("等待有效关节反馈"); return; }
  moveit::core::RobotState candidate(*actual_);
  for (size_t i = 0; i < variables_.size(); ++i) candidate.setVariablePosition(variables_[i], joint_edits_[i]->value());
  candidate.update();
  if (!candidate.satisfiesBounds(group_) || display_->getPlanningSceneRO()->isStateColliding(candidate, group_name_)) { invalidate(); notice("目标超限或碰撞"); return; }
  display_->setQueryGoalState(candidate); syncGoal(); notice("关节目标已更新");
}
void OperatorPanel::syncGoal() {
  if (!initialized_ || !display_ || display_->getCurrentPlanningGroup() != group_name_) return;
  const auto goal = display_->getQueryGoalState(); if (!goal) return;
  std::vector<double> q; goal->copyJointGroupPositions(group_, q);
  if (q.size() == last_goal_.size() && std::equal(q.begin(), q.end(), last_goal_.begin(), [](double a, double b) { return std::abs(a-b) < 1e-7; })) return;
  last_goal_ = q; invalidate();
  goal_valid_ = goal->satisfiesBounds(group_) && !display_->getPlanningSceneRO()->isStateColliding(*goal, group_name_);
  const auto t = goal->getGlobalLinkTransform(base_).inverse() * goal->getGlobalLinkTransform(tcp_);
  displayPose(1, t); const auto values = xyzrpy(t);
  for (size_t i = 0; i < 6; ++i) { QSignalBlocker b(pose_edits_[i]); pose_edits_[i]->setValue(values[i]); }
  for (size_t i = 0; i < q.size(); ++i) { QSignalBlocker b(joint_edits_[i]); joint_edits_[i]->setValue(q[i]); }
  goal_pub_->publish(stamped(t, base_));
  sensor_msgs::msg::JointState msg; msg.header.stamp = node_->now(); msg.name = variables_; msg.position = q; goal_joints_pub_->publish(msg);
  notice(goal_valid_ ? "目标已更新" : "目标超限或碰撞");
}
void OperatorPanel::applyLimits() {
  if (!initialized_ || !limits_supported_) { notice("限速模型不支持"); return; }
  cap_ = 1.0; double tcp_bound = 0;
  for (size_t i = 0; i < vmax_.size(); ++i) { cap_ = std::min(cap_, limit_edits_[i]->value()/vmax_[i]); tcp_bound += radii_[i]*vmax_[i]; }
  if (tcp_limit_->value() > 0 && tcp_bound > 0) cap_ = std::min(cap_, tcp_limit_->value()/tcp_bound);
  cap_ = std::floor(cap_*1e6)/1e6;
  if (cap_ < 1e-6) { limits_dirty_ = true; invalidate(); notice("限速低于支持精度"); return; }
  // Keep the user's request separate from the constrained value. Reusing the
  // previous effective value would make a low TCP cap permanently latch slow speed.
  syncing_ = true; native_scale_->setValue(std::min(requested_scale_->value(), cap_)); syncing_ = false;
  limits_dirty_ = false; invalidate(); publishLimits(); notice("限速已更新，需 Plan");
}
void OperatorPanel::publishLimits() {
  QJsonObject data; QJsonArray requested, effective, radii, names;
  const double scale = native_scale_->value();
  for (size_t i = 0; i < vmax_.size(); ++i) {
    requested.append(limit_edits_[i]->value()); effective.append(vmax_[i]*scale); radii.append(radii_[i]); names.append(QString::fromStdString(variables_[i]));
    joints_->item(i, 5)->setText(QString::number(vmax_[i]*scale, 'f', 6));
  }
  data["joint_names"] = names; data["requested_rad_s"] = requested; data["effective_rad_s"] = effective;
  data["jacobian_radius_bounds_m"] = radii; data["tcp_translation_m_s"] = tcp_limit_->value(); data["native_velocity_scaling"] = scale;
  data["requested_velocity_scaling"] = requested_scale_->value(); data["maximum_velocity_scaling"] = cap_;
  data["tcp_limit_enabled"] = tcp_limit_->value() > 0;
  data["scope"] = "this RViz native MotionPlanning only; conservative common scaling";
  std_msgs::msg::String msg; msg.data = QJsonDocument(data).toJson(QJsonDocument::Compact).toStdString(); limits_pub_->publish(msg);
  limits_label_->setText(QString("生效倍率 %1 / 约束上限 %2").arg(scale, 0, 'f', 6).arg(cap_, 0, 'f', 6));
}
void OperatorPanel::receiveObject(const geometry_msgs::msg::PoseStamped& p, bool truth) {
  if (truth != (object_source_->currentIndex() == 1)) return;
  object_ = p; object_received_ = std::chrono::steady_clock::now(); received_object_ = true;
}
void OperatorPanel::displayPose(int row, const Eigen::Isometry3d& t) {
  const auto values = xyzrpy(t); for (int c = 0; c < 6; ++c) poses_->item(row, c)->setText(QString::number(values[c], 'f', 5));
}
void OperatorPanel::notice(const QString& message) {
  status_->setText(message);
  if (status_pub_) { std_msgs::msg::String msg; msg.data = message.toStdString(); status_pub_->publish(msg); }
}
void OperatorPanel::tick() {
  executor_.spin_some(std::chrono::milliseconds(2));
  if (++ticks_ % 5) return;
  if (!bindDisplay()) return;
  if (!freshJoints()) {
    for (int c = 0; c < 6; ++c) poses_->item(0, c)->setText("反馈过期/缺失");
    for (size_t i = 0; i < variables_.size(); ++i) joints_->item(i, 1)->setText("--");
    guardNativeControls(); return;
  }
  for (size_t i = 0; i < last_joints_.name.size() && i < last_joints_.position.size(); ++i)
    if (std::find(model_->getVariableNames().begin(), model_->getVariableNames().end(), last_joints_.name[i]) != model_->getVariableNames().end() && std::isfinite(last_joints_.position[i])) actual_->setVariablePosition(last_joints_.name[i], last_joints_.position[i]);
  actual_->update(); const auto tcp_base = actual_->getGlobalLinkTransform(base_).inverse() * actual_->getGlobalLinkTransform(tcp_);
  displayPose(0, tcp_base); auto current = stamped(tcp_base, base_); current.header.stamp = last_joints_.header.stamp; current_pub_->publish(current);
  for (size_t i = 0; i < variables_.size(); ++i) joints_->item(i, 1)->setText(QString::number(actual_->getVariablePosition(variables_[i]), 'f', 5));
  try {
    const double age = (node_->now() - rclcpp::Time(object_.header.stamp, node_->get_clock()->get_clock_type())).seconds();
    if (!received_object_ || age < -0.1 || age > 2.0 || std::chrono::duration<double>(std::chrono::steady_clock::now()-object_received_).count() > 2.0) throw std::runtime_error("物体输入过期/缺失");
    auto base_object = inBase(object_); auto tcp_object = tcp_base.inverse()*base_object;
    displayPose(2, base_object); displayPose(3, tcp_object);
    auto b = stamped(base_object, base_); b.header.stamp = object_.header.stamp; object_base_pub_->publish(b);
    object_tcp_pub_->publish(stamped(tcp_object, tcp_));
  } catch (const std::exception&) {
    for (int row : {2, 3}) for (int c = 0; c < 6; ++c) poses_->item(row, c)->setText("输入/TF过期");
  }
  guardNativeControls();
}
}
PLUGINLIB_EXPORT_CLASS(rm65b_operator_panel::OperatorPanel, rviz_common::Panel)
