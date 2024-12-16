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

#include "delete_dialog.h"
#include <algorithm>
#include <QtWidgets>
using std::vector;
using std::string;


DeleteDialog::DeleteDialog(
  QWidget* parent)
: QDialog(parent)
{
  setWindowTitle("Warning");
  QVBoxLayout* warning_message_vbox_layout = new QVBoxLayout;
  warning_message_vbox_layout->addWidget(new QLabel(
      "This item might be linked to other entities and/or on other levels.\n "
      "Are you sure you want to delete this item?"));

  QHBoxLayout* bottom_buttons_layout = new QHBoxLayout;
  _ok_button = new QPushButton("OK", this);  // first button = [enter] button
  bottom_buttons_layout->addWidget(_ok_button);
  connect(
    _ok_button, &QAbstractButton::clicked,
    this, &DeleteDialog::ok_button_clicked);

  _cancel_button = new QPushButton("Cancel", this);
  bottom_buttons_layout->addWidget(_cancel_button);
  connect(
    _cancel_button, &QAbstractButton::clicked,
    this, &QDialog::reject);

  QVBoxLayout* vbox_layout = new QVBoxLayout;
  vbox_layout->addLayout(warning_message_vbox_layout);
  vbox_layout->addLayout(bottom_buttons_layout);

  setLayout(vbox_layout);
}

DeleteDialog::~DeleteDialog()
{
}

void DeleteDialog::ok_button_clicked()
{
  accept();
}