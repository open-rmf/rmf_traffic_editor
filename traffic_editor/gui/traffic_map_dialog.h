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

#ifndef TRAFFIC_MAP_DIALOG_H
#define TRAFFIC_MAP_DIALOG_H

#include <QDialog>
#include "project.h"
#include "traffic_map.h"
class QLineEdit;
class QComboBox;


class TrafficMapDialog : public QDialog
{
public:
  TrafficMapDialog(TrafficMap& _scenario);
  ~TrafficMapDialog();

private:
  TrafficMap& traffic_map;

  QLineEdit* name_line_edit;
  QLineEdit* path_line_edit;
  QPushButton* ok_button, * cancel_button;

private slots:
  void ok_button_clicked();
  void path_button_clicked();
};

#endif
