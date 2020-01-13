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

#include "lift_dialog.h"
#include <QtWidgets>
using std::vector;


LiftDialog::LiftDialog(QWidget *parent, Lift& lift)
: QDialog(parent),
  _lift(lift)
{
  QHBoxLayout *bottom_buttons_layout = new QHBoxLayout;
  _ok_button = new QPushButton("OK", this);  // first button = [enter] button
  bottom_buttons_layout->addWidget(_ok_button);
  connect(
      _ok_button, &QAbstractButton::clicked,
      this, &LiftDialog::ok_button_clicked);

  _cancel_button = new QPushButton("Cancel", this);
  bottom_buttons_layout->addWidget(_cancel_button);
  connect(
    _cancel_button, &QAbstractButton::clicked,
    this, &QDialog::reject);

  QVBoxLayout *fields_vbox = new QVBoxLayout;

  QHBoxLayout *name_hbox = new QHBoxLayout;
  name_hbox->addWidget(new QLabel("Name:"));

  _name_line_edit =
      new QLineEdit(QString::fromStdString(_lift.name), this);
  name_hbox->addWidget(_name_line_edit);

  fields_vbox->addLayout(name_hbox);

  QHBoxLayout *top_hbox_layout = new QHBoxLayout;
  top_hbox_layout->addLayout(fields_vbox);

  QVBoxLayout *vbox_layout = new QVBoxLayout;
  vbox_layout->addLayout(top_hbox_layout);
  // todo: some sort of separator (?)
  vbox_layout->addLayout(bottom_buttons_layout);

  setLayout(vbox_layout);

  _name_line_edit->setFocus(Qt::OtherFocusReason);
}

LiftDialog::~LiftDialog()
{
}

void LiftDialog::ok_button_clicked()
{
  if (_name_line_edit->text().isEmpty())
  {
    QMessageBox::critical(this, "Error", "Lift name is empty");
    return;
  }

  _lift.name = _name_line_edit->text().toStdString();

  accept();
}
