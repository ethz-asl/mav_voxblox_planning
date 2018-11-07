#include <stdio.h>
#include <functional>
#include <thread>

#include <QCheckBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QTimer>
#include <QVBoxLayout>

#include <geometry_msgs/Twist.h>
#include <mav_planning_msgs/PlannerService.h>
#include <ros/names.h>
#include <rviz/visualization_manager.h>
#include <std_srvs/Empty.h>

#include "mav_planning_rviz/edit_button.h"
#include "mav_planning_rviz/planning_panel.h"
#include "mav_planning_rviz/pose_widget.h"

namespace mav_planning_rviz {

PlanningPanel::PlanningPanel(QWidget* parent)
    : rviz::Panel(parent), nh_(ros::NodeHandle()), interactive_markers_(nh_) {
  createLayout();
}

void PlanningPanel::onInitialize() {
  interactive_markers_.initialize();
  interactive_markers_.setPoseUpdatedCallback(
      std::bind(&PlanningPanel::updateInteractiveMarkerPose, this,
                std::placeholders::_1));

  interactive_markers_.setFrameId(vis_manager_->getFixedFrame().toStdString());
  // Initialize all the markers.
  for (const auto& kv : pose_widget_map_) {
    mav_msgs::EigenTrajectoryPoint pose;
    kv.second->getPose(&pose);
    interactive_markers_.enableMarker(kv.first, pose);
  }
}

void PlanningPanel::createLayout() {
  QGridLayout* topic_layout = new QGridLayout;
  // Input the namespace.
  topic_layout->addWidget(new QLabel("Namespace:"), 0, 0);
  namespace_editor_ = new QLineEdit;
  topic_layout->addWidget(namespace_editor_, 0, 1);
  topic_layout->addWidget(new QLabel("Planner name:"), 1, 0);
  planner_name_editor_ = new QLineEdit;
  topic_layout->addWidget(planner_name_editor_, 1, 1);
  topic_layout->addWidget(new QLabel("Odometry topic:"), 2, 0);
  odometry_topic_editor_ = new QLineEdit;
  topic_layout->addWidget(odometry_topic_editor_, 2, 1);
  odometry_checkbox_ = new QCheckBox("Set start to odom");
  topic_layout->addWidget(odometry_checkbox_, 3, 0, 1, 2);

  // Start and goal poses.
  QGridLayout* start_goal_layout = new QGridLayout;

  // Minimums...
  start_goal_layout->setColumnMinimumWidth(0, 50);
  start_goal_layout->setColumnMinimumWidth(1, 245);
  start_goal_layout->setColumnMinimumWidth(2, 80);
  start_goal_layout->setRowMinimumHeight(0, 55);
  start_goal_layout->setRowMinimumHeight(1, 55);
  start_goal_layout->setColumnStretch(0, 1);
  start_goal_layout->setColumnStretch(1, 9);
  start_goal_layout->setColumnStretch(2, 3);

  start_pose_widget_ = new PoseWidget("start");
  goal_pose_widget_ = new PoseWidget("goal");
  EditButton* start_edit_button = new EditButton("start");
  EditButton* goal_edit_button = new EditButton("goal");
  registerPoseWidget(start_pose_widget_);
  registerPoseWidget(goal_pose_widget_);
  registerEditButton(start_edit_button);
  registerEditButton(goal_edit_button);

  start_goal_layout->addWidget(new QLabel("Start:"), 0, 0, Qt::AlignTop);
  start_goal_layout->addWidget(start_pose_widget_, 0, 1);
  start_goal_layout->addWidget(start_edit_button, 0, 2);
  start_goal_layout->addWidget(new QLabel("Goal:"), 1, 0, Qt::AlignTop);
  start_goal_layout->addWidget(goal_pose_widget_, 1, 1);
  start_goal_layout->addWidget(goal_edit_button, 1, 2);

  // Planner services and publications.
  QGridLayout* service_layout = new QGridLayout;
  planner_service_button_ = new QPushButton("Planner Service");
  publish_path_button_ = new QPushButton("Publish Path");
  waypoint_button_ = new QPushButton("Send Waypoint");
  controller_button_ = new QPushButton("Send To Controller");
  service_layout->addWidget(planner_service_button_, 0, 0);
  service_layout->addWidget(publish_path_button_, 0, 1);
  service_layout->addWidget(waypoint_button_, 1, 0);
  service_layout->addWidget(controller_button_, 1, 1);

  // First the names, then the start/goal, then service buttons.
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(topic_layout);
  layout->addLayout(start_goal_layout);
  layout->addLayout(service_layout);
  setLayout(layout);

  // Hook up connections.
  connect(namespace_editor_, SIGNAL(editingFinished()), this,
          SLOT(updateNamespace()));
  connect(planner_name_editor_, SIGNAL(editingFinished()), this,
          SLOT(updatePlannerName()));
  connect(odometry_topic_editor_, SIGNAL(editingFinished()), this,
          SLOT(updateOdometryTopic()));
  connect(planner_service_button_, SIGNAL(released()), this,
          SLOT(callPlannerService()));
  connect(publish_path_button_, SIGNAL(released()), this,
          SLOT(callPublishPath()));
  connect(waypoint_button_, SIGNAL(released()), this, SLOT(publishWaypoint()));
  connect(controller_button_, SIGNAL(released()), this,
          SLOT(publishToController()));
  connect(odometry_checkbox_, SIGNAL(stateChanged(int)), this,
          SLOT(trackOdometryStateChanged(int)));
}

void PlanningPanel::trackOdometryStateChanged(int state) {
  if (state == 0) {
    track_odometry_ = 0;
  } else {
    track_odometry_ = 1;
  }
}

void PlanningPanel::updateNamespace() {
  setNamespace(namespace_editor_->text());
}

// Set the topic name we are publishing to.
void PlanningPanel::setNamespace(const QString& new_namespace) {
  ROS_DEBUG_STREAM("Setting namespace from: " << namespace_.toStdString()
                                              << " to "
                                              << new_namespace.toStdString());
  // Only take action if the name has changed.
  if (new_namespace != namespace_) {
    namespace_ = new_namespace;
    Q_EMIT configChanged();

    std::string error;
    if (ros::names::validate(namespace_.toStdString(), error)) {
      waypoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
          namespace_.toStdString() + "/waypoint", 1, false);
      controller_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
          namespace_.toStdString() + "/command/pose", 1, false);
      odometry_sub_ = nh_.subscribe(
          namespace_.toStdString() + "/" + odometry_topic_.toStdString(), 1,
          &PlanningPanel::odometryCallback, this);
    }
  }
}

void PlanningPanel::updatePlannerName() {
  setPlannerName(planner_name_editor_->text());
}

// Set the topic name we are publishing to.
void PlanningPanel::setPlannerName(const QString& new_planner_name) {
  // Only take action if the name has changed.
  if (new_planner_name != planner_name_) {
    planner_name_ = new_planner_name;
    Q_EMIT configChanged();
  }
}

void PlanningPanel::updateOdometryTopic() {
  setOdometryTopic(odometry_topic_editor_->text());
}

// Set the topic name we are publishing to.
void PlanningPanel::setOdometryTopic(const QString& new_odometry_topic) {
  // Only take action if the name has changed.
  if (new_odometry_topic != odometry_topic_) {
    odometry_topic_ = new_odometry_topic;
    Q_EMIT configChanged();

    std::string error;
    if (ros::names::validate(namespace_.toStdString(), error)) {
      odometry_sub_ = nh_.subscribe(
          namespace_.toStdString() + "/" + odometry_topic_.toStdString(), 1,
          &PlanningPanel::odometryCallback, this);
    }
  }
}

void PlanningPanel::startEditing(const std::string& id) {
  // Make sure nothing else is being edited.
  if (!currently_editing_.empty()) {
    auto search = edit_button_map_.find(currently_editing_);
    if (search != edit_button_map_.end()) {
      search->second->finishEditing();
    }
  }
  currently_editing_ = id;
  // Get the current pose:
  auto search = pose_widget_map_.find(currently_editing_);
  if (search == pose_widget_map_.end()) {
    return;
  }
  // Update fixed frame (may have changed since last time):
  interactive_markers_.setFrameId(vis_manager_->getFixedFrame().toStdString());
  mav_msgs::EigenTrajectoryPoint pose;
  search->second->getPose(&pose);
  interactive_markers_.enableSetPoseMarker(pose);
  interactive_markers_.disableMarker(id);
}

void PlanningPanel::finishEditing(const std::string& id) {
  if (currently_editing_ == id) {
    currently_editing_.clear();
    interactive_markers_.disableSetPoseMarker();
  }
  auto search = pose_widget_map_.find(id);
  if (search == pose_widget_map_.end()) {
    return;
  }
  ros::spinOnce();
  mav_msgs::EigenTrajectoryPoint pose;
  search->second->getPose(&pose);
  interactive_markers_.enableMarker(id, pose);
}

void PlanningPanel::registerPoseWidget(PoseWidget* widget) {
  pose_widget_map_[widget->id()] = widget;
  connect(widget, SIGNAL(poseUpdated(const std::string&,
                                     mav_msgs::EigenTrajectoryPoint&)),
          this, SLOT(widgetPoseUpdated(const std::string&,
                                       mav_msgs::EigenTrajectoryPoint&)));
}

void PlanningPanel::registerEditButton(EditButton* button) {
  edit_button_map_[button->id()] = button;
  connect(button, SIGNAL(startedEditing(const std::string&)), this,
          SLOT(startEditing(const std::string&)));
  connect(button, SIGNAL(finishedEditing(const std::string&)), this,
          SLOT(finishEditing(const std::string&)));
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void PlanningPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
  config.mapSetValue("namespace", namespace_);
  config.mapSetValue("planner_name", planner_name_);
  config.mapSetValue("odometry_topic", odometry_topic_);
}

// Load all configuration data for this panel from the given Config object.
void PlanningPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
  QString topic;
  QString ns;
  if (config.mapGetString("namespace", &ns)) {
    namespace_editor_->setText(ns);
    updateNamespace();
  }
  if (config.mapGetString("planner_name", &planner_name_)) {
    planner_name_editor_->setText(planner_name_);
  }
  if (config.mapGetString("odometry_topic", &topic)) {
    odometry_topic_editor_->setText(topic);
    updateOdometryTopic();
  }
}

void PlanningPanel::updateInteractiveMarkerPose(
    const mav_msgs::EigenTrajectoryPoint& pose) {
  if (currently_editing_.empty()) {
    return;
  }
  auto search = pose_widget_map_.find(currently_editing_);
  if (search == pose_widget_map_.end()) {
    return;
  }
  search->second->setPose(pose);
}

void PlanningPanel::widgetPoseUpdated(const std::string& id,
                                      mav_msgs::EigenTrajectoryPoint& pose) {
  if (currently_editing_ == id) {
    interactive_markers_.setPose(pose);
  }
  interactive_markers_.updateMarkerPose(id, pose);
}

void PlanningPanel::callPlannerService() {
  std::string service_name =
      namespace_.toStdString() + "/" + planner_name_.toStdString() + "/plan";
  mav_msgs::EigenTrajectoryPoint start_point, goal_point;

  start_pose_widget_->getPose(&start_point);
  goal_pose_widget_->getPose(&goal_point);

  std::thread t([service_name, start_point, goal_point] {
    mav_planning_msgs::PlannerService req;
    mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(start_point,
                                                     &req.request.start_pose);
    mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(goal_point,
                                                     &req.request.goal_pose);

    try {
      ROS_DEBUG_STREAM("Service name: " << service_name);
      if (!ros::service::call(service_name, req)) {
        ROS_WARN_STREAM("Couldn't call service: " << service_name);
      }
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM("Service Exception: " << e.what());
    }
  });
  t.detach();
}

void PlanningPanel::callPublishPath() {
  std_srvs::Empty req;
  std::string service_name = namespace_.toStdString() + "/" +
                             planner_name_.toStdString() + "/publish_path";
  try {
    if (!ros::service::call(service_name, req)) {
      ROS_WARN_STREAM("Couldn't call service: " << service_name);
    }
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Service Exception: " << e.what());
  }
}

void PlanningPanel::publishWaypoint() {
  mav_msgs::EigenTrajectoryPoint goal_point;
  goal_pose_widget_->getPose(&goal_point);

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = vis_manager_->getFixedFrame().toStdString();
  mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(goal_point, &pose);

  ROS_DEBUG_STREAM("Publishing waypoint on "
                   << waypoint_pub_.getTopic()
                   << " subscribers: " << waypoint_pub_.getNumSubscribers());

  waypoint_pub_.publish(pose);
}

void PlanningPanel::publishToController() {
  mav_msgs::EigenTrajectoryPoint goal_point;
  goal_pose_widget_->getPose(&goal_point);

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = vis_manager_->getFixedFrame().toStdString();
  mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(goal_point, &pose);

  ROS_DEBUG_STREAM("Publishing controller goal on "
                   << controller_pub_.getTopic()
                   << " subscribers: " << controller_pub_.getNumSubscribers());

  controller_pub_.publish(pose);
}

void PlanningPanel::odometryCallback(const nav_msgs::Odometry& msg) {
  ROS_INFO_ONCE("Got odometry callback.");
  if (track_odometry_) {
    mav_msgs::EigenOdometry odometry;
    mav_msgs::eigenOdometryFromMsg(msg, &odometry);
    mav_msgs::EigenTrajectoryPoint point;
    point.position_W = odometry.position_W;
    point.orientation_W_B = odometry.orientation_W_B;
    pose_widget_map_["start"]->setPose(point);
    interactive_markers_.updateMarkerPose("start", point);
  }
}

}  // namespace mav_planning_rviz

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mav_planning_rviz::PlanningPanel, rviz::Panel)
