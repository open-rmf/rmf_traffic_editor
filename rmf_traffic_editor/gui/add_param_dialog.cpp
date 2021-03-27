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

#include "add_param_dialog.h"
#include <QtWidgets>


AddParamDialog::AddParamDialog(
  QWidget* parent,
  const std::vector<std::pair<std::string, Param::Type>>& _param_names)
: QDialog(parent),
  param_names(_param_names)
{
  ok_button = new QPushButton("OK", this);  // first button = [enter] button
  cancel_button = new QPushButton("Cancel", this);

  QHBoxLayout* name_hbox_layout = new QHBoxLayout;
  name_hbox_layout->addWidget(new QLabel("name:"));
  name_combo_box = new QComboBox;
  for (const auto& param_name : param_names)
    name_combo_box->addItem(QString::fromStdString(param_name.first));
  name_hbox_layout->addWidget(name_combo_box);

  QHBoxLayout* bottom_buttons_layout = new QHBoxLayout;
  bottom_buttons_layout->addWidget(cancel_button);
  bottom_buttons_layout->addWidget(ok_button);
  connect(
    ok_button, &QAbstractButton::clicked,
    this, &AddParamDialog::ok_button_clicked);
  connect(
    cancel_button, &QAbstractButton::clicked,
    this, &QDialog::reject);

  QVBoxLayout* vbox_layout = new QVBoxLayout;
  vbox_layout->addLayout(name_hbox_layout);
  // todo: some sort of separator (?)
  vbox_layout->addLayout(bottom_buttons_layout);

  setLayout(vbox_layout);
}

AddParamDialog::~AddParamDialog()
{
}

void AddParamDialog::ok_button_clicked()
{
  // todo: if this box becomes more complex in the future, validate it...
  accept();
}

std::string AddParamDialog::get_param_name() const
{
  return name_combo_box->currentText().toStdString();
}

Param::Type AddParamDialog::get_param_type() const
{
  // loop through the allowed_params to find the name that is selected
  // then return the type
  for (const auto& param_name : param_names)
  {
    if (name_combo_box->currentText().toStdString() != param_name.first)
      continue;
    return param_name.second;
  }
  return Param::Type::UNDEFINED;
}
