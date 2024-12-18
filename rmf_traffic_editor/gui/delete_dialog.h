/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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

#ifndef DELETE_DIALOG_H
#define DELETE_DIALOG_H

#include <QDialog>
#include <map>
#include "building.h"

class DeleteDialog : public QDialog
{
  Q_OBJECT

public:
  DeleteDialog(
    QWidget* parent,
    const Building& building,
    int level_idx,
    int selected_vertex_idx);
  ~DeleteDialog();

private:
  QPushButton* _ok_button, * _cancel_button;

private slots:
  void ok_button_clicked();
};

#endif
