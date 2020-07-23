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

#include "scenario_dialog.h"
#include <QtWidgets>
#include <utility>
using std::unique_ptr;


ScenarioDialog::ScenarioDialog(Scenario& _scenario)
: QDialog(),
  scenario(_scenario)
{
  setWindowTitle("Scenario Properties");
  ok_button = new QPushButton("OK", this);  // first button = [enter] button
  cancel_button = new QPushButton("Cancel", this);

  QHBoxLayout* name_hbox = new QHBoxLayout;
  name_hbox->addWidget(new QLabel("Scenario name:"));
  name_line_edit = new QLineEdit(QString::fromStdString(scenario.name));
  name_hbox->addWidget(name_line_edit);

  QHBoxLayout* scenario_hbox = new QHBoxLayout;
  scenario_path_line_edit = new QLineEdit(
    QString::fromStdString(scenario.filename));
  QPushButton* scenario_path_button = new QPushButton("Find...");
  connect(
    scenario_path_button,
    &QAbstractButton::clicked,
    this,
    &ScenarioDialog::scenario_path_button_clicked);

  scenario_hbox->addWidget(new QLabel("Scenario path:"));
  scenario_hbox->addWidget(scenario_path_line_edit);
  scenario_hbox->addWidget(scenario_path_button);

  QHBoxLayout* bottom_buttons_hbox = new QHBoxLayout;
  bottom_buttons_hbox->addWidget(cancel_button);
  bottom_buttons_hbox->addWidget(ok_button);
  connect(
    ok_button,
    &QAbstractButton::clicked,
    this,
    &ScenarioDialog::ok_button_clicked);
  connect(
    cancel_button,
    &QAbstractButton::clicked,
    this,
    &QDialog::reject);

  QVBoxLayout* top_vbox = new QVBoxLayout;
  top_vbox->addLayout(name_hbox);
  top_vbox->addLayout(scenario_hbox);
  // todo: some sort of separator (?)
  top_vbox->addLayout(bottom_buttons_hbox);

  setLayout(top_vbox);
}

ScenarioDialog::~ScenarioDialog()
{
}

void ScenarioDialog::ok_button_clicked()
{
  scenario.name = name_line_edit->text().toStdString();

  if (scenario_path_line_edit->text().isEmpty())
  {
    QMessageBox::critical(
      this,
      "Scenario filename not provided",
      "Scenario filename must be defined. Otherwise click [Cancel].");
    return;
  }

  if (!scenario_path_line_edit->text().endsWith(".scenario.yaml"))
  {
    QMessageBox::critical(
      this,
      "Bad scenario filename",
      "Scenario filename must end in .scenario.yaml");
    return;
  }

  scenario.filename = scenario_path_line_edit->text().toStdString();

  accept();
}

void ScenarioDialog::scenario_path_button_clicked()
{
  QFileDialog file_dialog(this, "Scenario File");
  //file_dialog.setFileMode(QFileDialog::ExistingFile);
  file_dialog.setNameFilter("*.scenario.yaml");
  if (file_dialog.exec() != QDialog::Accepted)
    return;// user clicked 'cancel' in the QFileDialog
  const QString filename = file_dialog.selectedFiles().first();

  // if the scenario filename has changed, load it.
  const std::string previous_scenario_filename = scenario.filename;

  scenario_path_line_edit->setText(
    QDir::current().relativeFilePath(filename));

  scenario.filename = scenario_path_line_edit->text().toStdString();

  if (scenario.filename != previous_scenario_filename)
  {
    if (QFileInfo(filename).exists() && !scenario.load())
    {
      QMessageBox::critical(
        this,
        "Unable to load scenario file",
        "Unable to load scenario file. Check filename?");
    }
  }
}
