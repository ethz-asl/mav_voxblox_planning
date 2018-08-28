#ifndef MAV_PLANNING_RVIZ_EDIT_BUTTON_H_
#define MAV_PLANNING_RVIZ_EDIT_BUTTON_H_

#ifndef Q_MOC_RUN
#include <mav_msgs/eigen_mav_msgs.h>
#include <QPushButton>
#include <QWidget>
#endif

namespace mav_planning_rviz {

// This is a little widget that allows pose input.
class EditButton : public QWidget {
  Q_OBJECT
 public:
  explicit EditButton(const std::string& id, QWidget* parent = 0);

  std::string id() const { return id_; }
  void setId(const std::string& id) { id_ = id; }

  virtual QSize sizeHint() const {
    return edit_button_->sizeHint();
  }

 public Q_SLOTS:
  void startEditing();
  void finishEditing();
  void toggle();

  // Communicate to the main process what this button is doing:
 Q_SIGNALS:
  void startedEditing(const std::string& id);
  void finishedEditing(const std::string& id);

 protected:
  // Set up the layout, only called by the constructor.
  void createButton();

  // QT stuff:
  QPushButton* edit_button_;

  // Other state:
  // This is the ID that binds the button to the pose widget.
  std::string id_;
  bool editing_;
};

}  // end namespace mav_planning_rviz

#endif  // MAV_PLANNING_RVIZ_EDIT_BUTTON_H_
