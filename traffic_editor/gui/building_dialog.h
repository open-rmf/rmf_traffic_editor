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

#ifndef BUILDING_DIALOG_H
#define BUILDING_DIALOG_H

#include <QDialog>
#include "traffic_editor/building.h"
class QLineEdit;
class QComboBox;


class BuildingDialog : public QDialog
{
public:
  BuildingDialog(Building& building);
  ~BuildingDialog();

private:
  Building& _building;

  QLineEdit* _building_name_line_edit;
  QComboBox* _reference_floor_combo_box;
  QPushButton* _ok_button, * _cancel_button;

private slots:
  void ok_button_clicked();
};

#endif
