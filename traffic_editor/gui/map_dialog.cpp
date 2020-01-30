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

#include "map_dialog.h"
#include <QtWidgets>


MapDialog::MapDialog(Map& map)
: QDialog(), _map(map)
{
  _ok_button = new QPushButton("OK", this);  // first button = [enter] button
  _cancel_button = new QPushButton("Cancel", this);

  QHBoxLayout *building_name_hbox = new QHBoxLayout;
  building_name_hbox->addWidget(new QLabel("Building name:"));
  _building_name_line_edit = new QLineEdit(
      QString::fromStdString(map.building_name),
      this);
  building_name_hbox->addWidget(_building_name_line_edit);

  /////////////////////////////////////////////

  // TODO: combo box to select the reference level used to derive the
  // coordinate system of the generated model(s)

  /////////////////////////////////////////////

  QHBoxLayout *bottom_buttons_hbox = new QHBoxLayout;
  bottom_buttons_hbox->addWidget(_cancel_button);
  bottom_buttons_hbox->addWidget(_ok_button);
  connect(
      _ok_button, &QAbstractButton::clicked,
      this, &MapDialog::ok_button_clicked);
  connect(
      _cancel_button, &QAbstractButton::clicked,
      this, &QDialog::reject);

  QVBoxLayout *top_vbox = new QVBoxLayout;

  top_vbox->addLayout(building_name_hbox);
  // todo: some sort of separator (?)
  top_vbox->addLayout(bottom_buttons_hbox);

  setLayout(top_vbox);
}

MapDialog::~MapDialog()
{
}

void MapDialog::ok_button_clicked()
{
  _map.building_name = _building_name_line_edit->text().toStdString();
  accept();
}
