#ifndef MAV_PLANNING_RVIZ_PLANNING_PANEL_H_
#define MAV_PLANNING_RVIZ_PLANNING_PANEL_H_

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/config.h>
#include "mav_planning_rviz/planning_interactive_markers.h"
#include "mav_planning_rviz/pose_widget.h"
#endif

class QLineEdit;
namespace mav_planning_rviz {

class PlanningPanel : public QWidget {
  Q_OBJECT
 public:
  explicit PlanningPanel(QWidget* parent = 0);

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.
  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

  // Next come a couple of public Qt slots.
 public Q_SLOTS:
  void setNamespace(const QString& new_namespace);

  // Here we declare some internal slots.
 protected Q_SLOTS:
  // Updates the namespace from the editor.
  void updateNamespace();


  // Some settings for QT to display this correctly:
  virtual QSize sizeHint() const;

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
