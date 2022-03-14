/*
 * Copyright (C) 2019-2021 Open Source Robotics Foundation
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

#ifndef LEVEL_DIALOG_H
#define LEVEL_DIALOG_H

#include <QDialog>
#include "building.h"
#include "level.h"
class QLineEdit;


class LevelDialog : public QDialog
{
public:
  LevelDialog(Level& level, Building& building);
  ~LevelDialog();

private:
  Level& building_level;
  Building& building;

  QLineEdit* name_line_edit, * drawing_filename_line_edit;
  QLineEdit* x_line_edit, * y_line_edit;
  QLineEdit* elevation_line_edit;
  QPushButton* drawing_filename_button;
  QPushButton* ok_button, * cancel_button;

  void enable_dimensions(const bool enable);

private slots:
  void drawing_filename_button_clicked();
  void ok_button_clicked();
  void drawing_filename_line_edited(const QString& text);
};

#endif
