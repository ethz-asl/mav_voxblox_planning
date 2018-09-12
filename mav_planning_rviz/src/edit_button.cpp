#include <QApplication>
#include <QDesktopWidget>
#include <QHeaderView>
#include <QTableView>
#include <QTableWidget>

#include "mav_planning_rviz/edit_button.h"

namespace mav_planning_rviz {

EditButton::EditButton(const std::string& id, QWidget* parent)
    : QWidget(parent), id_(id), editing_(false) {
  createButton();
}

void EditButton::createButton() {
  edit_button_ = new QPushButton("Edit", this);
  edit_button_->setAutoFillBackground(true);
  // edit_button_->setFlat(true);
  finishEditing();
  // Connect button signal to appropriate slot
  connect(edit_button_, SIGNAL(released()), this, SLOT(toggle()));
}

void EditButton::toggle() {
  if (!editing_) {
    startEditing();
  } else {
    finishEditing();
  }
}

void EditButton::startEditing() {
  editing_ = true;
  edit_button_->setText("Finish");
  edit_button_->setStyleSheet(
      "background-color: rgb(204, 255, 179); color: rgb(0, 0, 0); outline: "
      "none;");
  Q_EMIT startedEditing(id_);
}

void EditButton::finishEditing() {
  editing_ = false;
  edit_button_->setText("Edit");
  edit_button_->setStyleSheet(
      "background-color: rgb(255, 255, 204); color: rgb(0, 0, 0); outline: "
      "none;");
  Q_EMIT finishedEditing(id_);

}

}  // namespace mav_planning_rviz
