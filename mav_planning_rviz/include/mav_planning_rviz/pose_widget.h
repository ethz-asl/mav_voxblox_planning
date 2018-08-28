#ifndef MAV_PLANNING_RVIZ_POSE_WIDGET_H_
#define MAV_PLANNING_RVIZ_POSE_WIDGET_H_

#ifndef Q_MOC_RUN
#include <mav_msgs/eigen_mav_msgs.h>
#include <QStringList>
#include <QTableWidget>
#include <QLineEdit>
#include <QItemDelegate>
#endif

class QLineEdit;
namespace mav_planning_rviz {

// This is a little widget that allows pose input.
class PoseWidget : public QWidget {
  Q_OBJECT
 public:
  explicit PoseWidget(QWidget* parent = 0);

  void getPose(mav_msgs::EigenTrajectoryPoint* point) const;

 public Q_SLOTS:
 protected Q_SLOTS:

 protected:
  // Set up the layout, only called by the constructor.
  void createTable();

  // QT stuff:
  QTableWidget* table_widget_;

  // QT state:
  QStringList table_headers_;
};

class DoubleTableDelegate : public QItemDelegate {
 public:
  QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option,
                        const QModelIndex& index) const {
    QLineEdit* line_edit = new QLineEdit(parent);
    // Set validator
    QDoubleValidator* validator = new QDoubleValidator(line_edit);
    line_edit->setValidator(validator);
    return line_edit;
  }
};

}  // end namespace mav_planning_rviz

#endif  // MAV_PLANNING_RVIZ_POSE_WIDGET_H_
