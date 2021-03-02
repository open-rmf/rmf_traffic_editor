/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef PREFERENCES_DIALOG_H
#define PREFERENCES_DIALOG_H

#include <QDialog>
class QLineEdit;
class QCheckBox;


class PreferencesDialog : public QDialog
{
public:
  PreferencesDialog(QWidget* parent);
  ~PreferencesDialog();

private:
  QLineEdit* thumbnail_path_line_edit;
  QPushButton* thumbnail_path_button;
  QCheckBox* open_previous_building_checkbox;
  QPushButton* ok_button, * cancel_button;

private slots:
  void thumbnail_path_button_clicked();
  void ok_button_clicked();
};

#endif
