#include <ros/ros.h>
#include <QApplication>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QtGui>

#include "mav_planning_rviz/planning_panel.h"

using namespace mav_planning_rviz;

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "planning_panel", ros::init_options::NoSigintHandler);
  QApplication app(argc, argv);
  QWidget window;
  window.resize(600, 400);
  window.show();
  window.setWindowTitle(QApplication::translate("toplevel", "Planning Panel"));

  PlanningPanel planning_panel(&window);
  planning_panel.show();

  ros::spinOnce();
  return app.exec();
}
