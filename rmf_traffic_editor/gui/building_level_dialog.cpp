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

#include "building_level_dialog.h"
#include <QtWidgets>


BuildingLevelDialog::BuildingLevelDialog(BuildingLevel& _level,
  Building& _building)
: building_level(_level), building(_building)
{
  ok_button = new QPushButton("OK", this);  // first button = [enter] button
  cancel_button = new QPushButton("Cancel", this);

  name_line_edit = new QLineEdit(
    QString::fromStdString(building_level.name), this);
  QHBoxLayout* name_hbox = new QHBoxLayout;
  name_hbox->addWidget(new QLabel("name:"));
  name_hbox->addWidget(name_line_edit);

  elevation_line_edit = new QLineEdit(
    QString::number(building_level.elevation));
  QHBoxLayout* elevation_hbox = new QHBoxLayout;
  elevation_hbox->addWidget(new QLabel("elevation:"));
  elevation_hbox->addWidget(elevation_line_edit);

  drawing_filename_line_edit = new QLineEdit(
    QString::fromStdString(building_level.drawing_filename),
    this);
  drawing_filename_button = new QPushButton("Find...", this);
  QHBoxLayout* drawing_filename_hbox = new QHBoxLayout;
  drawing_filename_hbox->addWidget(new QLabel("drawing:"));
  drawing_filename_hbox->addWidget(drawing_filename_line_edit);
  drawing_filename_hbox->addWidget(drawing_filename_button);
  connect(
    drawing_filename_button,
    &QAbstractButton::clicked,
    this,
    &BuildingLevelDialog::drawing_filename_button_clicked);
  connect(
    drawing_filename_line_edit,
    &QLineEdit::textEdited,
    this,
    &BuildingLevelDialog::drawing_filename_line_edited);

  QHBoxLayout* instr_hbox = new QHBoxLayout;
  instr_hbox->addWidget(
    new QLabel(
      "Explicit dimensions are only needed if drawing is not provided:"));

  QHBoxLayout* x_hbox = new QHBoxLayout;
  x_line_edit = new QLineEdit(QString::number(building_level.x_meters), this);
  x_hbox->addWidget(new QLabel("x dimension (meters):"));
  x_hbox->addWidget(x_line_edit);

  QHBoxLayout* y_hbox = new QHBoxLayout;
  y_line_edit = new QLineEdit(QString::number(building_level.y_meters), this);
  y_hbox->addWidget(new QLabel("y dimension (meters):"));
  y_hbox->addWidget(y_line_edit);

  flattened_x_offset_line_edit =
    new QLineEdit(QString::number(building_level.flattened_x_offset));
  QHBoxLayout* flattened_x_offset_hbox = new QHBoxLayout;
  flattened_x_offset_hbox->addWidget(
    new QLabel("flattened x offset (meters)"));
  flattened_x_offset_hbox->addWidget(flattened_x_offset_line_edit);

  flattened_y_offset_line_edit =
    new QLineEdit(QString::number(building_level.flattened_y_offset));
  QHBoxLayout* flattened_y_offset_hbox = new QHBoxLayout;
  flattened_y_offset_hbox->addWidget(
    new QLabel("flattened y offset (meters)"));
  flattened_y_offset_hbox->addWidget(flattened_y_offset_line_edit);

  QHBoxLayout* bottom_buttons_hbox = new QHBoxLayout;
  bottom_buttons_hbox->addWidget(cancel_button);
  bottom_buttons_hbox->addWidget(ok_button);
  connect(
    ok_button,
    &QAbstractButton::clicked,
    this,
    &BuildingLevelDialog::ok_button_clicked);
  connect(
    cancel_button,
    &QAbstractButton::clicked,
    this,
    &QDialog::reject);

  QVBoxLayout* top_vbox = new QVBoxLayout;
  top_vbox->addLayout(name_hbox);
  top_vbox->addLayout(elevation_hbox);
  top_vbox->addLayout(drawing_filename_hbox);
  top_vbox->addLayout(instr_hbox);
  top_vbox->addLayout(x_hbox);
  top_vbox->addLayout(y_hbox);
  top_vbox->addLayout(flattened_x_offset_hbox);
  top_vbox->addLayout(flattened_y_offset_hbox);
  // todo: some sort of separator (?)
  top_vbox->addLayout(bottom_buttons_hbox);

  setLayout(top_vbox);

  enable_dimensions(building_level.drawing_filename.empty());
}

BuildingLevelDialog::~BuildingLevelDialog()
{
}

void BuildingLevelDialog::drawing_filename_button_clicked()
{
  QFileDialog file_dialog(this, "Find Drawing");
  file_dialog.setFileMode(QFileDialog::ExistingFile);
  file_dialog.setNameFilter("*.png");
  if (file_dialog.exec() != QDialog::Accepted)
  {
    if (drawing_filename_line_edit->text().isEmpty())
      enable_dimensions(true);
    return;  // user clicked 'cancel'
  }
  const QString filename = file_dialog.selectedFiles().first();
  if (!QFileInfo(filename).exists())
  {
    QMessageBox::critical(
      this,
      "Drawing file does not exist",
      "File does not exist.");
    if (drawing_filename_line_edit->text().isEmpty())
      enable_dimensions(true);
    return;
  }
  drawing_filename_line_edit->setText(
    QDir::current().relativeFilePath(filename));
  enable_dimensions(false);
}

void BuildingLevelDialog::ok_button_clicked()
{
  if (!drawing_filename_line_edit->text().isEmpty())
  {
    // make sure the drawing file exists
    if (!QFileInfo(drawing_filename_line_edit->text()).exists())
    {
      QMessageBox::critical(
        this,
        "If supplied, drawing filename must exist",
        "If supplied, drawing filename must exist");
      return;
    }
  }
  /*
  // todo: figure out how to test for valid numeric values;
  // this doesn't work but there must be a similar function somewhere
  if (!x_line_edit->text().isNumber() || !y_line_edit->text().isNumber()) {
    QMessageBox::critical(
        this,
        "X and Y dimensions must be numbers",
        "X and Y dimensions must be numbers");
    return;
  }
  */
  if (name_line_edit->text().isEmpty())
  {
    QMessageBox::critical(
      this,
      "Name must not be empty",
      "Name must not be empty");
    return;
  }
  auto original_name = building_level.name;
  building_level.name = name_line_edit->text().toStdString();
  building_level.elevation = elevation_line_edit->text().toDouble();
  for (size_t i = 0; i < building.lifts.size(); i ++)
  {
    if (original_name != building_level.name)
    {
      if (building.lifts[i].level_doors.find(original_name) !=
        building.lifts[i].level_doors.end())
      {
        building.lifts[i].level_doors[building_level.name] =
          building.lifts[i].level_doors[original_name];
        building.lifts[i].level_doors.erase(original_name);
      }
    }
    if (building.lifts[i].highest_floor == original_name)
    {
      building.lifts[i].highest_floor = building_level.name;
      building.lifts[i].highest_elevation = building_level.elevation;
    }
    if (building.lifts[i].lowest_floor == original_name)
    {
      building.lifts[i].lowest_floor = building_level.name;
      building.lifts[i].lowest_elevation = building_level.elevation;
    }
  }
  building_level.drawing_filename =
    drawing_filename_line_edit->text().toStdString();
  if (building_level.drawing_filename.empty())
  {
    building_level.x_meters = x_line_edit->text().toDouble();
    building_level.y_meters = y_line_edit->text().toDouble();
  }
  else
  {
    building_level.x_meters = 0.0;
    building_level.y_meters = 0.0;
  }

  building_level.flattened_x_offset =
    flattened_x_offset_line_edit->text().toDouble();
  building_level.flattened_y_offset =
    flattened_y_offset_line_edit->text().toDouble();

  building_level.calculate_scale();
  accept();
}

void BuildingLevelDialog::enable_dimensions(const bool enable)
{
  if (enable)
  {
    x_line_edit->setEnabled(true);
    y_line_edit->setEnabled(true);
  }
  else
  {
    x_line_edit->setText("10");
    y_line_edit->setText("10");
    x_line_edit->setEnabled(false);
    y_line_edit->setEnabled(false);
  }
}

void BuildingLevelDialog::drawing_filename_line_edited(const QString& text)
{
  enable_dimensions(text.isEmpty());
}
