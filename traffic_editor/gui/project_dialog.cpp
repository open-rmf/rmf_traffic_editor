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

#include "project_dialog.h"
#include <QtWidgets>


ProjectDialog::ProjectDialog(Project& _project)
: QDialog(),
  project(_project)
{
  setWindowTitle("Project Properties");
  ok_button = new QPushButton("OK", this);  // first button = [enter] button
  cancel_button = new QPushButton("Cancel", this);

  QHBoxLayout* name_hbox = new QHBoxLayout;
  name_hbox->addWidget(new QLabel("Project name:"));
  name_line_edit = new QLineEdit(QString::fromStdString(project.name));
  name_hbox->addWidget(name_line_edit);

  QHBoxLayout* building_hbox = new QHBoxLayout;
  building_path_line_edit = new QLineEdit(
    QString::fromStdString(project.building.filename));
  QPushButton* building_path_button = new QPushButton("Find...");
  connect(
    building_path_button,
    &QAbstractButton::clicked,
    this,
    &ProjectDialog::building_path_button_clicked);

  building_hbox->addWidget(new QLabel("Building path:"));
  building_hbox->addWidget(building_path_line_edit);
  building_hbox->addWidget(building_path_button);

  QHBoxLayout* bottom_buttons_hbox = new QHBoxLayout;
  bottom_buttons_hbox->addWidget(cancel_button);
  bottom_buttons_hbox->addWidget(ok_button);
  connect(
    ok_button,
    &QAbstractButton::clicked,
    this,
    &ProjectDialog::ok_button_clicked);
  connect(
    cancel_button,
    &QAbstractButton::clicked,
    this,
    &QDialog::reject);

  QVBoxLayout* top_vbox = new QVBoxLayout;
  top_vbox->addLayout(name_hbox);
  top_vbox->addLayout(building_hbox);
  // todo: some sort of separator (?)
  top_vbox->addLayout(bottom_buttons_hbox);

  setLayout(top_vbox);
}

ProjectDialog::~ProjectDialog()
{
}

void ProjectDialog::ok_button_clicked()
{
  project.name = name_line_edit->text().toStdString();

  // if the building filename has changed, load it.
  const std::string previous_building_filename = project.building.filename;
  project.building.filename = building_path_line_edit->text().toStdString();
  if (project.building.filename != previous_building_filename)
    project.building.load_yaml_file();

  accept();
}

void ProjectDialog::building_path_button_clicked()
{
  QFileDialog file_dialog(this, "Building File");
  file_dialog.setFileMode(QFileDialog::ExistingFile);
  file_dialog.setNameFilter("*.building.yaml");
  if (file_dialog.exec() != QDialog::Accepted)
    return;// user clicked 'cancel' in the QFileDialog
  const QString filename = file_dialog.selectedFiles().first();
  if (!QFileInfo(filename).exists())  // is this check even needed?
  {
    QMessageBox::critical(
      this,
      "Building file does not exist",
      "File does not exist.");
    return;
  }
  // todo: probably should change to the path of this project before
  // calculating the relative path. This is already done
  // implicitly, but maybe should be more explicit here.
  building_path_line_edit->setText(
    QDir::current().relativeFilePath(filename));
}
