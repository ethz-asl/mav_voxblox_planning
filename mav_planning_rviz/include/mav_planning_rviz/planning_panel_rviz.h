#ifndef MAV_PLANNING_RVIZ_PLANNING_PANEL_RVIZ_H_
#define MAV_PLANNING_RVIZ_PLANNING_PANEL_RVIZ_H_

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/panel.h>
#include "mav_planning_rviz/planning_panel.h"
#endif

class QLineEdit;
namespace mav_planning_rviz {

// class DriveWidget;

class PlanningPanelRviz : public rviz::Panel {
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
  explicit PlanningPanelRviz(QWidget* parent = 0);

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

 private:
  PlanningPanel planning_panel_;
};

}  // end namespace mav_planning_rviz

#endif  // MAV_PLANNING_RVIZ_PLANNING_PANEL_RVIZ_H_
