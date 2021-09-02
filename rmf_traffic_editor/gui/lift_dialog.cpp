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
#include <cfloat>
#include <QtWidgets>
using std::vector;


LiftDialog::LiftDialog(Lift& lift, Building& building)
: QDialog(),
  _lift(lift),
  _building(building)
{
  setWindowTitle("Lift Properties");
  for (const auto& level : building.levels)
    _level_names.push_back(QString::fromStdString(level.name));

  QHBoxLayout* bottom_buttons_hbox = new QHBoxLayout;
  _ok_button = new QPushButton("OK", this);  // first button = [enter] button
  bottom_buttons_hbox->addWidget(_ok_button);
  connect(
    _ok_button, &QAbstractButton::clicked,
    this, &LiftDialog::ok_button_clicked);

  _cancel_button = new QPushButton("Cancel", this);
  bottom_buttons_hbox->addWidget(_cancel_button);
  connect(
    _cancel_button,
    &QAbstractButton::clicked,
    this,
    &QDialog::reject);

  QHBoxLayout* name_hbox = new QHBoxLayout;
  name_hbox->addWidget(new QLabel("Name:"));
  _name_line_edit =
    new QLineEdit(QString::fromStdString(_lift.name), this);
  connect(
    _name_line_edit,
    &QLineEdit::textEdited,
    [this](const QString& text)
    {
      _lift.name = text.toStdString();
      update_lift_view();
      emit redraw();
    });
  name_hbox->addWidget(_name_line_edit);

  QHBoxLayout* ref_name_hbox = new QHBoxLayout;
  ref_name_hbox->addWidget(new QLabel("Reference floor:"));
  _reference_floor_combo_box = new QComboBox;
  for (const QString& level_name : _level_names)
    _reference_floor_combo_box->addItem(level_name);
  _reference_floor_combo_box->setCurrentText(
    QString::fromStdString(_lift.reference_floor_name));
  connect(
    _reference_floor_combo_box,
    &QComboBox::currentTextChanged,
    [this](const QString& text)
    {
      _lift.reference_floor_name = text.toStdString();
      emit redraw();
    });
  ref_name_hbox->addWidget(_reference_floor_combo_box);

  QHBoxLayout* init_floor_hbox = new QHBoxLayout;
  init_floor_hbox->addWidget(new QLabel("Initial floor:"));
  _initial_floor_combo_box = new QComboBox;
  for (const QString& level_name : _level_names)
    _initial_floor_combo_box->addItem(level_name);
  _initial_floor_combo_box->setCurrentText(
    QString::fromStdString(_lift.initial_floor_name));
  connect(
    _initial_floor_combo_box,
    &QComboBox::currentTextChanged,
    [this](const QString& text)
    {
      _lift.initial_floor_name = text.toStdString();
      emit redraw();
    });
  init_floor_hbox->addWidget(_initial_floor_combo_box);

  QHBoxLayout* highest_name_hbox = new QHBoxLayout;
  highest_name_hbox->addWidget(new QLabel("Highest floor:"));
  _highest_floor_combo_box = new QComboBox;
  for (const QString& level_name : _level_names)
    _highest_floor_combo_box->addItem(level_name);
  _highest_floor_combo_box->addItem("");  // empty string for not specifying
  _highest_floor_combo_box->setCurrentText(
    QString::fromStdString(_lift.highest_floor));
  connect(
    _highest_floor_combo_box,
    &QComboBox::currentTextChanged,
    [this](const QString& text)
    {
      _lift.highest_floor = text.toStdString();
      if (_lift.highest_floor.empty())
        _lift.highest_elevation = DBL_MAX;
      else
      {
        for (const auto& level : _building.levels)
        {
          if (level.name == _lift.highest_floor)
          {
            _lift.highest_elevation = level.elevation;
            break;
          }
        }
      }
      update_level_table();
      emit redraw();
    });
  highest_name_hbox->addWidget(_highest_floor_combo_box);

  QHBoxLayout* lowest_name_hbox = new QHBoxLayout;
  lowest_name_hbox->addWidget(new QLabel("Lowest floor:"));
  _lowest_floor_combo_box = new QComboBox;
  for (const QString& level_name : _level_names)
    _lowest_floor_combo_box->addItem(level_name);
  _lowest_floor_combo_box->addItem("");
  _lowest_floor_combo_box->setCurrentText(
    QString::fromStdString(_lift.lowest_floor));
  connect(
    _lowest_floor_combo_box,
    &QComboBox::currentTextChanged,
    [this](const QString& text)
    {
      _lift.lowest_floor = text.toStdString();
      if (_lift.lowest_floor.empty())
        _lift.lowest_elevation = -DBL_MAX;
      else
      {
        for (const auto& level : _building.levels)
        {
          if (level.name == _lift.lowest_floor)
          {
            _lift.lowest_elevation = level.elevation;
            break;
          }
        }
      }
      update_level_table();
      emit redraw();
    });
  lowest_name_hbox->addWidget(_lowest_floor_combo_box);

  QHBoxLayout* x_hbox = new QHBoxLayout;
  x_hbox->addWidget(new QLabel("X:"));
  _x_line_edit =
    new QLineEdit(QString::number(_lift.x), this);
  connect(
    _x_line_edit,
    &QLineEdit::textEdited,
    [this](const QString& text)
    {
      _lift.x = text.toDouble();
      update_lift_wps();
    });
  x_hbox->addWidget(_x_line_edit);

  QHBoxLayout* y_hbox = new QHBoxLayout;
  y_hbox->addWidget(new QLabel("Y:"));
  _y_line_edit =
    new QLineEdit(QString::number(_lift.y), this);
  connect(
    _y_line_edit,
    &QLineEdit::textEdited,
    [this](const QString& text)
    {
      _lift.y = text.toDouble();
      update_lift_wps();
    });
  y_hbox->addWidget(_y_line_edit);

  QHBoxLayout* yaw_hbox = new QHBoxLayout;
  yaw_hbox->addWidget(new QLabel("Yaw:"));
  _yaw_line_edit =
    new QLineEdit(QString::number(_lift.yaw), this);
  connect(
    _yaw_line_edit,
    &QLineEdit::textEdited,
    [this](const QString& text)
    {
      _lift.yaw = text.toDouble();
      update_lift_view();
      emit redraw();
    });
  yaw_hbox->addWidget(_yaw_line_edit);

  QHBoxLayout* width_hbox = new QHBoxLayout;
  width_hbox->addWidget(new QLabel("Cabin width:"));
  _width_line_edit =
    new QLineEdit(QString::number(_lift.width), this);
  connect(
    _width_line_edit,
    &QLineEdit::textEdited,
    [this](const QString& text)
    {
      _lift.width = text.toDouble();
      update_lift_view();
      emit redraw();
    });
  width_hbox->addWidget(_width_line_edit);

  QHBoxLayout* depth_hbox = new QHBoxLayout;
  depth_hbox->addWidget(new QLabel("Cabin depth:"));
  _depth_line_edit =
    new QLineEdit(QString::number(_lift.depth), this);
  connect(
    _depth_line_edit,
    &QLineEdit::textEdited,
    [this](const QString& text)
    {
      _lift.depth = text.toDouble();
      update_lift_view();
      emit redraw();
    });
  depth_hbox->addWidget(_depth_line_edit);

  QHBoxLayout* plugin_hbox = new QHBoxLayout;
  plugin_hbox->addWidget(new QLabel("Plugin:"));
  _plugin_yes_radio_button = new QRadioButton("Yes");
  _plugin_no_radio_button = new QRadioButton("No");
  if (_lift.plugins)
    _plugin_yes_radio_button->setChecked(true);
  else
    _plugin_no_radio_button->setChecked(true);

  plugin_hbox->addWidget(_plugin_yes_radio_button);
  plugin_hbox->addWidget(_plugin_no_radio_button);

  QHBoxLayout* add_wp_hbox = new QHBoxLayout;
  _add_wp_button = new QPushButton("Add lift waypoints", this);
  add_wp_hbox->addWidget(_add_wp_button);
  connect(
    _add_wp_button, &QAbstractButton::clicked,
    this, &LiftDialog::update_lift_wps);

  _level_table = new QTableWidget;
  _level_table->setMinimumSize(200, 200);
  _level_table->verticalHeader()->setVisible(false);
  _level_table->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);

  _level_table->setHorizontalHeaderItem(0, new QTableWidgetItem("Level name"));
  /*
  _level_table->horizontalHeader()->setSectionResizeMode(
      0, QHeaderView::Stretch);
  */

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

  QVBoxLayout* left_vbox = new QVBoxLayout;
  left_vbox->addLayout(name_hbox);
  left_vbox->addLayout(ref_name_hbox);
  left_vbox->addLayout(highest_name_hbox);
  left_vbox->addLayout(lowest_name_hbox);
  left_vbox->addLayout(init_floor_hbox);
  left_vbox->addLayout(x_hbox);
  left_vbox->addLayout(y_hbox);
  left_vbox->addLayout(yaw_hbox);
  left_vbox->addLayout(width_hbox);
  left_vbox->addLayout(depth_hbox);
  left_vbox->addLayout(plugin_hbox);
  left_vbox->addLayout(add_wp_hbox);
  left_vbox->addWidget(_level_table);

  QVBoxLayout* right_vbox = new QVBoxLayout;

  _lift_scene = new QGraphicsScene;

  _lift_view = new QGraphicsView;
  _lift_view->setScene(_lift_scene);
  _lift_view->setMinimumSize(400, 400);
  right_vbox->addWidget(_lift_view, 1);

  right_vbox->addWidget(_door_table);

  QHBoxLayout* top_hbox = new QHBoxLayout;
  top_hbox->addLayout(left_vbox);
  top_hbox->addLayout(right_vbox, 1);

  QVBoxLayout* top_vbox = new QVBoxLayout;
  top_vbox->addLayout(top_hbox);
  // todo: some sort of separator (?)
  top_vbox->addLayout(bottom_buttons_hbox);

  setLayout(top_vbox);

  _name_line_edit->setFocus(Qt::OtherFocusReason);

  update_door_table();
  update_level_table();

  connect(
    _door_table, &QTableWidget::cellChanged,
    this, &LiftDialog::door_table_cell_changed);

  update_lift_view();
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

  if (_lift.lowest_elevation > _lift.highest_elevation)
  {
    QMessageBox::critical(this, "Error", "Lowest floor above highest floor");
    return;
  }

  _lift.name = _name_line_edit->text().toStdString();
  _lift.reference_floor_name =
    _reference_floor_combo_box->currentText().toStdString();
  _lift.highest_floor = _highest_floor_combo_box->currentText().toStdString();
  _lift.lowest_floor = _lowest_floor_combo_box->currentText().toStdString();
  _lift.initial_floor_name =
    _initial_floor_combo_box->currentText().toStdString();

  _lift.x = _x_line_edit->text().toDouble();
  _lift.y = _y_line_edit->text().toDouble();
  _lift.yaw = _yaw_line_edit->text().toDouble();

  _lift.width = _width_line_edit->text().toDouble();
  _lift.depth = _depth_line_edit->text().toDouble();

  if (_plugin_yes_radio_button->isChecked())
    _lift.plugins = true;
  else if (_plugin_no_radio_button->isChecked())
    _lift.plugins = false;

  // grab all the level-door checkbox matrix states and save them
  for (int level_row = 0; level_row < _level_table->rowCount(); level_row++)
  {
    const std::string level_name =
      _level_table->item(level_row, 0)->text().toStdString();
    _lift.level_doors.erase(level_name);
    for (int door_col = 1; door_col < _level_table->columnCount(); door_col++)
    {
      const std::string door_name =
        _level_table->horizontalHeaderItem(door_col)->text().toStdString();
      const QWidget* widget = _level_table->cellWidget(level_row, door_col);
      const QCheckBox* checkbox = qobject_cast<const QCheckBox*>(widget);
      if (checkbox)
      {
        const bool checked = checkbox->isChecked();
        printf("level %s door %s: %d\n",
          level_name.c_str(),
          door_name.c_str(),
          checked ? 1 : 0);
        if (checked)
          _lift.level_doors[level_name].push_back(door_name);
      }
      else
      {
        printf("level %s door %s: indeterminate state!\n",
          level_name.c_str(),
          door_name.c_str());
      }
    }
  }
  update_lift_view();
  emit redraw();
  accept();
}

void LiftDialog::update_lift_wps()
{
  const QPointF from_point = QPointF(_lift.x, _lift.y);
  QPointF to_point;

  bool found = false;
  for (std::size_t level_idx = 0; level_idx < _level_names.size(); level_idx++)
  {
    const std::string level_name = _level_names[level_idx].toStdString();
    // Vertices will only be generated on levels that the lift is serving (has
    // a door opening on that level)
    if (_lift.level_doors[level_name].size() != 0)
    {
      _building.transform_between_levels(
        _lift.reference_floor_name,
        from_point,
        _building.levels[level_idx].name,
        to_point);
      found = false;

      for (auto& v : _building.levels[level_idx].vertices)
      {
        auto it = v.params.find("lift_cabin");
        if ((it != v.params.end()) && (it->second.value_string == _lift.name))
        {
          v.x = to_point.x();
          v.y = to_point.y();
          found = true;
        }
      }
      if (!found)
      {
        _building.add_vertex(level_idx, to_point.x(), to_point.y());
        _building.levels[level_idx].vertices.back().params["lift_cabin"] =
          _lift.name;
      }
    }
  }
  emit redraw();
}

void LiftDialog::update_door_table()
{
  _door_table->setRowCount(1 + _lift.doors.size());
  for (std::size_t i = 0; i < _lift.doors.size(); i++)
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
    QComboBox* type_box = new QComboBox;
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
  QPushButton* add_button = new QPushButton("Add...", this);
  _door_table->setCellWidget(last_row_idx, 5, add_button);
  connect(
    add_button,
    &QAbstractButton::clicked,
    [this]()
    {
      LiftDoor door;
      door.name = "name";
      door.door_type = LiftDoor::DOUBLE_SLIDING;
      _lift.doors.push_back(door);
      update_door_table();
      update_lift_view();
    });
}

void LiftDialog::set_door_cell(
  const int row,
  const int col,
  const QString& text)
{
  _door_table->setItem(row, col, new QTableWidgetItem(text));
}

void LiftDialog::update_level_table()
{
  _level_table->setColumnCount(1 + static_cast<int>(_lift.doors.size()));
  _level_table->setHorizontalHeaderItem(0, new QTableWidgetItem("Level"));
  for (std::size_t door_idx = 0; door_idx < _lift.doors.size(); door_idx++)
  {
    _level_table->setHorizontalHeaderItem(
      door_idx + 1,
      new QTableWidgetItem(
        QString::fromStdString(_lift.doors[door_idx].name)));
  }
  //blockSignals(true);
  _level_table->setRowCount(_level_names.size());
  for (std::size_t level_idx = 0; level_idx < _level_names.size(); level_idx++)
  {
    const QString& level_name = _level_names[level_idx];
    QTableWidgetItem* name_item = new QTableWidgetItem(level_name);
    name_item->setFlags(name_item->flags() & ~Qt::ItemIsEditable);
    _level_table->setItem(level_idx, 0, name_item);

    for (std::size_t door_idx = 0; door_idx < _lift.doors.size(); door_idx++)
    {
      QCheckBox* checkbox = new QCheckBox;
      checkbox->setStyleSheet("margin-left: 50%; margin-right: 50%");
      if (_lift.level_door_opens(
          level_name.toStdString(),
          _lift.doors[door_idx].name,
          _building.levels))
        checkbox->setChecked(true);
      _level_table->setCellWidget(level_idx, door_idx + 1, checkbox);
    }
  }
  //blockSignals(false);
}

void LiftDialog::door_table_cell_changed(int row, int col)
{
  // printf("door_table_cell_changed(%d, %d)\n", row, col);
  if (row >= static_cast<int>(_lift.doors.size()))
  {
    printf("invalid door row: %d\n", row);
    return;  // let's not crash
  }

  // If a door name was changed, we need to update the options shown in all
  // the level_table combo boxes
  if (col == 0)  // name
  {
    _lift.doors[row].name = _door_table->item(row, col)->text().toStdString();
    update_level_table();
  }
  else if (col == 1)
  {
    // todo: door type
  }
  else if (col == 2) // x
    _lift.doors[row].x = _door_table->item(row, col)->text().toDouble();
  else if (col == 3) // y
    _lift.doors[row].y = _door_table->item(row, col)->text().toDouble();
  else if (col == 4) // orientation
    _lift.doors[row].motion_axis_orientation =
      _door_table->item(row, col)->text().toDouble();
  else if (col == 5) // width
    _lift.doors[row].width = _door_table->item(row, col)->text().toDouble();

  update_lift_view();
  emit redraw();
}

void LiftDialog::update_lift_view()
{
  _lift_scene->clear();
  _lift.draw(_lift_scene, 0.01, std::string(), _lift.lowest_elevation, false);
}
