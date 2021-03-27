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

#include "traffic_map_dialog.h"
#include <QtWidgets>


TrafficMapDialog::TrafficMapDialog(TrafficMap& _traffic_map)
: QDialog(),
  traffic_map(_traffic_map)
{
  setWindowTitle("Traffic Map Properties");
  ok_button = new QPushButton("OK", this);  // first button = [enter] button
  cancel_button = new QPushButton("Cancel", this);

  QHBoxLayout* name_hbox = new QHBoxLayout;
  name_hbox->addWidget(new QLabel("Traffic map name:"));
  name_line_edit = new QLineEdit(QString::fromStdString(traffic_map.name));
  name_hbox->addWidget(name_line_edit);

  QHBoxLayout* path_hbox = new QHBoxLayout;
  path_line_edit =
    new QLineEdit(QString::fromStdString(traffic_map.filename));
  QPushButton* path_button = new QPushButton("Find...");
  connect(
    path_button,
    &QAbstractButton::clicked,
    this,
    &TrafficMapDialog::path_button_clicked);

  path_hbox->addWidget(new QLabel("Traffic map path:"));
  path_hbox->addWidget(path_line_edit);
  path_hbox->addWidget(path_button);

  QHBoxLayout* bottom_buttons_hbox = new QHBoxLayout;
  bottom_buttons_hbox->addWidget(cancel_button);
  bottom_buttons_hbox->addWidget(ok_button);
  connect(
    ok_button,
    &QAbstractButton::clicked,
    this,
    &TrafficMapDialog::ok_button_clicked);
  connect(
    cancel_button,
    &QAbstractButton::clicked,
    this,
    &QDialog::reject);

  QVBoxLayout* top_vbox = new QVBoxLayout;
  top_vbox->addLayout(name_hbox);
  top_vbox->addLayout(path_hbox);
  // todo: some sort of separator (?)
  top_vbox->addLayout(bottom_buttons_hbox);

  setLayout(top_vbox);
}

TrafficMapDialog::~TrafficMapDialog()
{
}

void TrafficMapDialog::ok_button_clicked()
{
  traffic_map.name = name_line_edit->text().toStdString();

  if (path_line_edit->text().isEmpty())
  {
    QMessageBox::critical(
      this,
      "Filename not provided",
      "Filename must be defined. Otherwise click [Cancel].");
    return;
  }

  if (!path_line_edit->text().endsWith(".traffic_map.yaml"))
  {
    QMessageBox::critical(
      this,
      "Bad filename",
      "Filename must end in .traffic_map.yaml");
    return;
  }

  traffic_map.filename = path_line_edit->text().toStdString();

  accept();
}

void TrafficMapDialog::path_button_clicked()
{
  QFileDialog file_dialog(this, "Traffic Map File");
  //file_dialog.setFileMode(QFileDialog::ExistingFile);
  file_dialog.setNameFilter("*.traffic_map.yaml");
  if (file_dialog.exec() != QDialog::Accepted)
    return;// user clicked 'cancel' in the QFileDialog
  const QString filename = file_dialog.selectedFiles().first();

  // if the filename has changed, load it.
  const std::string previous_filename = traffic_map.filename;

  path_line_edit->setText(QDir::current().relativeFilePath(filename));

  traffic_map.filename = path_line_edit->text().toStdString();

  if (traffic_map.filename != previous_filename)
  {
    if (QFileInfo(filename).exists() && !traffic_map.load_file())
    {
      QMessageBox::critical(
        this,
        "Unable to load file",
        "Unable to load file. Check filename?");
    }
  }
}
