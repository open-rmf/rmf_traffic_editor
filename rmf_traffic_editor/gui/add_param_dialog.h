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

#ifndef ADD_PARAM_DIALOG_H
#define ADD_PARAM_DIALOG_H

#include <vector>
#include <QDialog>
#include "param.h"
class QComboBox;

class AddParamDialog : public QDialog
{
public:
  AddParamDialog(
    QWidget* parent,
    const std::vector<std::pair<std::string, Param::Type>>& param_names);
  ~AddParamDialog();

  std::string get_param_name() const;
  Param::Type get_param_type() const;

private:
  QComboBox* name_combo_box;
  QPushButton* ok_button, * cancel_button;
  const std::vector<std::pair<std::string, Param::Type>>& param_names;

private slots:
  void ok_button_clicked();
};

#endif
