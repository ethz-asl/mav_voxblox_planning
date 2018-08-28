#include <QApplication>
#include <QtGui>
#include <QVBoxLayout>

#include "mav_planning_rviz/pose_widget.h"

using namespace mav_planning_rviz;

int main(int argc, char* argv[]) {
  QApplication app(argc, argv);
  QWidget window;
  window.resize(300, 200);
  window.show();
  window.setWindowTitle(
      QApplication::translate("toplevel", "Top-level widget"));

  QVBoxLayout* layout = new QVBoxLayout;
  PoseWidget* pose_widget = new PoseWidget(&window);
  layout->addWidget(pose_widget);
  window.setLayout(layout);

  return app.exec();
}
