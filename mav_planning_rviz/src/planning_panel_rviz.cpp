#include <stdio.h>

#include <rviz/panel.h>

#include "mav_planning_rviz/planning_panel_rviz.h"

namespace mav_planning_rviz {

PlanningPanelRviz::PlanningPanelRviz(QWidget* parent) : rviz::Panel(parent) {
  planning_panel_.show();
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void PlanningPanelRviz::save(rviz::Config config) const {
  rviz::Panel::save(config);
  planning_panel_.save(config);
}

// Load all configuration data for this panel from the given Config object.
void PlanningPanelRviz::load(const rviz::Config& config) {
  rviz::Panel::load(config);
  planning_panel_.load(config);
}

}  // end namespace mav_planning_rviz

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mav_planning_rviz::PlanningPanelRviz, rviz::Panel)
