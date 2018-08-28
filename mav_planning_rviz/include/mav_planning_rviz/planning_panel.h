#ifndef MAV_PLANNING_RVIZ_PLANNING_PANEL_H_
#define MAV_PLANNING_RVIZ_PLANNING_PANEL_H_

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#include "mav_planning_rviz/planning_interactive_markers.h"
#include "mav_planning_rviz/pose_widget.h"
#endif

class QLineEdit;
namespace mav_planning_rviz {

// class DriveWidget;

class PlanningPanel : public rviz::Panel {
  // This class uses Qt slots and is a subclass of QObject, so it needs
  // the Q_OBJECT macro.
  Q_OBJECT
 public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  explicit PlanningPanel(QWidget* parent = 0);

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

  // Next come a couple of public Qt slots.
 public Q_SLOTS:
  void setNamespace(const QString& new_namespace);

  // Here we declare some internal slots.
 protected Q_SLOTS:
  // Updates the namespace from the editor.
  void updateNamespace();

  // Then we finish up with protected member variables.
 protected:
  // Set up the layout, only called by the constructor.
  void createLayout();

  // ROS Stuff:
  ros::NodeHandle nh_;

  // QT stuff:
  QLineEdit* namespace_editor_;
  PoseWidget* start_pose_widget_;
  PoseWidget* goal_pose_widget_;

  // QT state:
  QString namespace_;

  // ROS state:
  PlanningInteractiveMarkers interactive_markers_;
};

}  // end namespace mav_planning_rviz

#endif  // MAV_PLANNING_RVIZ_PLANNING_PANEL_H_
