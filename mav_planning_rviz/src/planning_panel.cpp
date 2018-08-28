#include <stdio.h>

#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QTimer>
#include <QVBoxLayout>

#include <geometry_msgs/Twist.h>
#include <rviz/visualization_manager.h>

#include "mav_planning_rviz/edit_button.h"
#include "mav_planning_rviz/planning_panel.h"
#include "mav_planning_rviz/pose_widget.h"

namespace mav_planning_rviz {

PlanningPanel::PlanningPanel(QWidget* parent)
    : rviz::Panel(parent), nh_(ros::NodeHandle()), interactive_markers_(nh_) {
  createLayout();
}

void PlanningPanel::createLayout() {
  QHBoxLayout* topic_layout = new QHBoxLayout;
  // Input the namespace.
  topic_layout->addWidget(new QLabel("Namespace:"));
  namespace_editor_ = new QLineEdit;
  topic_layout->addWidget(namespace_editor_);

  // Start and goal poses.
  QGridLayout* start_goal_layout = new QGridLayout;
  start_pose_widget_ = new PoseWidget("start");
  goal_pose_widget_ = new PoseWidget("goal");
  start_goal_layout->addWidget(new QLabel("Start:"), 0, 0);
  start_goal_layout->addWidget(start_pose_widget_, 0, 1);
  start_goal_layout->addWidget(new QLabel("Goal:"), 1, 0);
  start_goal_layout->addWidget(goal_pose_widget_, 1, 1);
  start_goal_layout->setColumnStretch(0, 1);
  start_goal_layout->setColumnStretch(1, 20);

  // Lay out the topic field above the control widget.
  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(topic_layout);
  layout->addLayout(start_goal_layout);
  setLayout(layout);

  connect(namespace_editor_, SIGNAL(editingFinished()), this,
          SLOT(updateNamespace()));
}

void PlanningPanel::updateNamespace() {
  setNamespace(namespace_editor_->text());
}

// Set the topic name we are publishing to.
void PlanningPanel::setNamespace(const QString& new_namespace) {
  // Only take action if the name has changed.
  if (new_namespace != namespace_) {
    namespace_ = new_namespace;
    Q_EMIT configChanged();
  }

  ROS_INFO("Current time: %f", vis_manager_->getROSTime());
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void PlanningPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
  config.mapSetValue("namespace", namespace_);
}

// Load all configuration data for this panel from the given Config object.
void PlanningPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
  QString topic;
  if (config.mapGetString("namespace", &namespace_)) {
    namespace_editor_->setText(namespace_);
    updateNamespace();
  }
}

}  // end namespace mav_planning_rviz

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mav_planning_rviz::PlanningPanel, rviz::Panel)
