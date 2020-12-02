/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifndef LIFT_DIALOG_H
#define LIFT_DIALOG_H

#include <vector>

#include <QDialog>
#include <QObject>
#include <QRadioButton>

#include "traffic_editor/lift.h"
#include "traffic_editor/building.h"

class QLineEdit;
class QLabel;
class QTableWidget;
class QComboBox;


class LiftDialog : public QDialog
{
  Q_OBJECT

public:
  LiftDialog(Lift& lift, Building& building);
  ~LiftDialog();

private:
  Lift& _lift;
  Building& _building;

  std::vector<QString> _level_names;

  QLineEdit* _name_line_edit;
  QComboBox* _reference_floor_combo_box;
  QComboBox* _highest_floor_combo_box;
  QComboBox* _lowest_floor_combo_box;
  QComboBox* _initial_floor_combo_box;
  QLineEdit* _x_line_edit;
  QLineEdit* _y_line_edit;
  QLineEdit* _yaw_line_edit;
  QLineEdit* _width_line_edit;
  QLineEdit* _depth_line_edit;
  QRadioButton* _plugin_yes_radio_button;
  QRadioButton* _plugin_no_radio_button;

  QTableWidget* _door_table;
  QTableWidget* _level_table;

  QGraphicsView* _lift_view;
  QGraphicsScene* _lift_scene;

  QPushButton* _ok_button, * _cancel_button;
  QPushButton* _add_wp_button;

  void update_door_table();
  void set_door_cell(const int row, const int col, const QString& text);
  void door_table_cell_changed(int row, int col);

  void update_level_table();
  void update_lift_view();

  void update_lift_wps();

private slots:
  void ok_button_clicked();

signals:
  void redraw();
};

#endif
