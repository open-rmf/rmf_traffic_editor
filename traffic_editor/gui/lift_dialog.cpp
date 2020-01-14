/*
 * Copyright (C) 2019-2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "lift_dialog.h"
#include <QtWidgets>
using std::vector;


LiftDialog::LiftDialog(Lift& lift, const Map& map)
: QDialog(),
  _lift(lift)
{
  for (const Level& level : map.levels)
    _level_names.push_back(QString::fromStdString(level.name));

  QHBoxLayout *bottom_buttons_hbox = new QHBoxLayout;
  _ok_button = new QPushButton("OK", this);  // first button = [enter] button
  bottom_buttons_hbox->addWidget(_ok_button);
  connect(
      _ok_button, &QAbstractButton::clicked,
      this, &LiftDialog::ok_button_clicked);

  _cancel_button = new QPushButton("Cancel", this);
  bottom_buttons_hbox->addWidget(_cancel_button);
  connect(
    _cancel_button, &QAbstractButton::clicked,
    this, &QDialog::reject);

  QHBoxLayout *name_hbox = new QHBoxLayout;
  name_hbox->addWidget(new QLabel("Name:"));
  _name_line_edit =
      new QLineEdit(QString::fromStdString(_lift.name), this);
  name_hbox->addWidget(_name_line_edit);

  QHBoxLayout *ref_name_hbox = new QHBoxLayout;
  ref_name_hbox->addWidget(new QLabel("Reference floor:"));
  _reference_floor_combo_box = new QComboBox;
  for (const QString& level_name : _level_names)
    _reference_floor_combo_box->addItem(level_name);
  _reference_floor_combo_box->setCurrentText(
      QString::fromStdString(_lift.reference_floor_name));
  ref_name_hbox->addWidget(_reference_floor_combo_box);

  QHBoxLayout *x_hbox = new QHBoxLayout;
  x_hbox->addWidget(new QLabel("X:"));
  _x_line_edit =
      new QLineEdit(QString::number(_lift.x), this);
  x_hbox->addWidget(_x_line_edit);

  QHBoxLayout *y_hbox = new QHBoxLayout;
  y_hbox->addWidget(new QLabel("Y:"));
  _y_line_edit =
      new QLineEdit(QString::number(_lift.y), this);
  y_hbox->addWidget(_y_line_edit);

  QHBoxLayout *yaw_hbox = new QHBoxLayout;
  yaw_hbox->addWidget(new QLabel("Yaw:"));
  _yaw_line_edit =
      new QLineEdit(QString::number(_lift.yaw), this);
  yaw_hbox->addWidget(_yaw_line_edit);

  QHBoxLayout *width_hbox = new QHBoxLayout;
  width_hbox->addWidget(new QLabel("Cabin width:"));
  _width_line_edit = 
      new QLineEdit(QString::number(_lift.width), this);
  width_hbox->addWidget(_width_line_edit);

  QHBoxLayout *depth_hbox = new QHBoxLayout;
  depth_hbox->addWidget(new QLabel("Cabin depth:"));
  _depth_line_edit = 
      new QLineEdit(QString::number(_lift.depth), this);
  depth_hbox->addWidget(_depth_line_edit);

  _level_table = new QTableWidget;
  _level_table->setMinimumSize(200, 200);
  _level_table->verticalHeader()->setVisible(false);
  _level_table->setColumnCount(2);
  _level_table->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);

  _level_table->setHorizontalHeaderItem(0, new QTableWidgetItem("Level name"));
  /*
  _level_table->horizontalHeader()->setSectionResizeMode(
      0, QHeaderView::Stretch);
  */
  _level_table->setHorizontalHeaderItem(1, new QTableWidgetItem("Door name"));

  _door_table = new QTableWidget;
  _door_table->setMinimumSize(400, 200);
  _door_table->verticalHeader()->setVisible(false);
  _door_table->setColumnCount(6);
  _door_table->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);

  _door_table->setHorizontalHeaderItem(0, new QTableWidgetItem("Door name"));
  _door_table->horizontalHeader()->setSectionResizeMode(
      0, QHeaderView::Stretch);

  _door_table->setHorizontalHeaderItem(1, new QTableWidgetItem("Door type"));
  _door_table->horizontalHeader()->setSectionResizeMode(
      1, QHeaderView::ResizeToContents);

  _door_table->setHorizontalHeaderItem(2, new QTableWidgetItem("X"));
  _door_table->horizontalHeader()->setSectionResizeMode(
      2, QHeaderView::ResizeToContents);

  _door_table->setHorizontalHeaderItem(3, new QTableWidgetItem("Y"));
  _door_table->horizontalHeader()->setSectionResizeMode(
      3, QHeaderView::ResizeToContents);

  _door_table->setHorizontalHeaderItem(4, new QTableWidgetItem("Orientation"));
  _door_table->horizontalHeader()->setSectionResizeMode(
      4, QHeaderView::ResizeToContents);

  _door_table->setHorizontalHeaderItem(5, new QTableWidgetItem("Width"));
  _door_table->horizontalHeader()->setSectionResizeMode(
      5, QHeaderView::ResizeToContents);

  _door_table->verticalHeader()->setSectionResizeMode(
      QHeaderView::ResizeToContents);

  QVBoxLayout *left_vbox = new QVBoxLayout;
  left_vbox->addLayout(name_hbox);
  left_vbox->addLayout(ref_name_hbox);
  left_vbox->addLayout(x_hbox);
  left_vbox->addLayout(y_hbox);
  left_vbox->addLayout(yaw_hbox);
  left_vbox->addLayout(width_hbox);
  left_vbox->addLayout(depth_hbox);
  left_vbox->addWidget(_level_table);

  QVBoxLayout *right_vbox = new QVBoxLayout;

  lift_preview_widget = new LiftPreviewWidget(_lift);
  lift_preview_widget->setMinimumSize(400, 400);
  right_vbox->addWidget(lift_preview_widget, 1);

  right_vbox->addWidget(_door_table);

  QHBoxLayout *top_hbox = new QHBoxLayout;
  top_hbox->addLayout(left_vbox);
  top_hbox->addLayout(right_vbox, 1);

  QVBoxLayout *top_vbox = new QVBoxLayout;
  top_vbox->addLayout(top_hbox);
  // todo: some sort of separator (?)
  top_vbox->addLayout(bottom_buttons_hbox);

  setLayout(top_vbox);

  _name_line_edit->setFocus(Qt::OtherFocusReason);

  update_door_table();
  update_level_table();

  // todo: listen to door_table QTableWidget::cellChanged signal
  // and update the combo box item names in the level->door map

  adjustSize();
}

LiftDialog::~LiftDialog()
{
}

void LiftDialog::ok_button_clicked()
{
  if (_name_line_edit->text().isEmpty())
  {
    QMessageBox::critical(this, "Error", "Lift name is empty");
    return;
  }

  _lift.name = _name_line_edit->text().toStdString();

  accept();
}

void LiftDialog::update_door_table()
{
  blockSignals(true);
  _door_table->setRowCount(1 + _lift.doors.size());
  for (size_t i = 0; i < _lift.doors.size(); i++)
  {
    const LiftDoor& door = _lift.doors[i];  // save some typing
    set_door_cell(i, 0, QString::fromStdString(door.name));

    // set the numeric fields
    set_door_cell(i, 2, QString::number(door.x));
    set_door_cell(i, 3, QString::number(door.y));
    set_door_cell(i, 4, QString::number(door.motion_axis_orientation));
    _door_table->setCellWidget(i, 5, nullptr);
    set_door_cell(i, 5, QString::number(door.width));

    // create a drop-down list for the door type
    QComboBox *type_box = new QComboBox;
    type_box->addItem("<undefined>", QVariant(0));
    type_box->addItem("Single sliding", QVariant(1));
    type_box->addItem("Double sliding", QVariant(2));
    type_box->addItem("Single telescoping", QVariant(3));
    type_box->addItem("Double telescoping", QVariant(4));
    type_box->setCurrentIndex(static_cast<int>(door.door_type));

    _door_table->setCellWidget(i, 1, type_box);
  }

  // we'll use the last row for the "Add" button
  const int last_row_idx = static_cast<int>(_lift.doors.size());
  _door_table->setCellWidget(last_row_idx, 0, nullptr);
  _door_table->setCellWidget(last_row_idx, 1, nullptr);
  _door_table->setCellWidget(last_row_idx, 2, nullptr);
  _door_table->setCellWidget(last_row_idx, 3, nullptr);
  _door_table->setCellWidget(last_row_idx, 4, nullptr);
  QPushButton *add_button = new QPushButton("Add...", this);
  _door_table->setCellWidget(last_row_idx, 5, add_button);
  connect(
      add_button, &QAbstractButton::clicked,
      [this]()
      {
        LiftDoor door;
        door.name = "name";
        door.door_type = LiftDoor::DOUBLE_SLIDING;
        _lift.doors.push_back(door);
        update_door_table();
      });

  update_level_table();

  blockSignals(false);
}

void LiftDialog::set_door_cell(
    const int row,
    const int col,
    const QString &text)
{
  _door_table->setItem(row, col, new QTableWidgetItem(text));
}

void LiftDialog::update_level_table()
{
  blockSignals(true);
  _level_table->setRowCount(_level_names.size());
  for (size_t i = 0; i < _level_names.size(); i++)
  {
    QTableWidgetItem *name_item = new QTableWidgetItem(_level_names[i]);
    name_item->setFlags(name_item->flags() & ~Qt::ItemIsEditable);
    _level_table->setItem(i, 0, name_item);

    QComboBox *door_name_box = new QComboBox;
    door_name_box->addItem(QString());  // empty string = lift doesn't stop
    for (const LiftDoor& door : _lift.doors)
      door_name_box->addItem(QString::fromStdString(door.name));
    _level_table->setCellWidget(i, 1, door_name_box);
  }
  blockSignals(false);
}
