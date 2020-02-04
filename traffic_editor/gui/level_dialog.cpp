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

#include "level_dialog.h"
#include <QtWidgets>


LevelDialog::LevelDialog(Level &level)
: _level(level)
{
  _ok_button = new QPushButton("OK", this);  // first button = [enter] button
  _cancel_button = new QPushButton("Cancel", this);

  _name_line_edit = new QLineEdit(QString::fromStdString(_level.name), this);
  QHBoxLayout *name_hbox = new QHBoxLayout;
  name_hbox->addWidget(new QLabel("name:"));
  name_hbox->addWidget(_name_line_edit);

  _elevation_line_edit = new QLineEdit(
      QString::number(_level.elevation));  //, 'f', 2));
  QHBoxLayout *elevation_hbox = new QHBoxLayout;
  elevation_hbox->addWidget(new QLabel("elevation:"));
  elevation_hbox->addWidget(_elevation_line_edit);

  _drawing_filename_line_edit = new QLineEdit(
      QString::fromStdString(_level.drawing_filename),
      this);
  _drawing_filename_button = new QPushButton("Find...", this);
  QHBoxLayout *drawing_filename_hbox = new QHBoxLayout;
  drawing_filename_hbox->addWidget(new QLabel("drawing:"));
  drawing_filename_hbox->addWidget(_drawing_filename_line_edit);
  drawing_filename_hbox->addWidget(_drawing_filename_button);
  connect(
      _drawing_filename_button, &QAbstractButton::clicked,
      this, &LevelDialog::drawing_filename_button_clicked);
  connect(
      _drawing_filename_line_edit,
      &QLineEdit::textEdited,
      this,
      &LevelDialog::drawing_filename_line_edited);

  QHBoxLayout *instr_hbox = new QHBoxLayout;
  instr_hbox->addWidget(
      new QLabel(
          "Explicit dimensions are only needed if drawing is not provided:"));

  QHBoxLayout *x_hbox = new QHBoxLayout;
  _x_line_edit = new QLineEdit(QString::number(_level.x_meters), this);
  x_hbox->addWidget(new QLabel("x dimension (meters):"));
  x_hbox->addWidget(_x_line_edit);

  QHBoxLayout *y_hbox = new QHBoxLayout;
  _y_line_edit = new QLineEdit(QString::number(_level.y_meters), this);
  y_hbox->addWidget(new QLabel("y dimension (meters):"));
  y_hbox->addWidget(_y_line_edit);

  QHBoxLayout *bottom_buttons_hbox = new QHBoxLayout;
  bottom_buttons_hbox->addWidget(_cancel_button);
  bottom_buttons_hbox->addWidget(_ok_button);
  connect(
      _ok_button, &QAbstractButton::clicked,
      this, &LevelDialog::ok_button_clicked);
  connect(
      _cancel_button, &QAbstractButton::clicked,
      this, &QDialog::reject);

  QVBoxLayout *top_vbox = new QVBoxLayout;
  top_vbox->addLayout(name_hbox);
  top_vbox->addLayout(elevation_hbox);
  top_vbox->addLayout(drawing_filename_hbox);
  top_vbox->addLayout(instr_hbox);
  top_vbox->addLayout(x_hbox);
  top_vbox->addLayout(y_hbox);
  // todo: some sort of separator (?)
  top_vbox->addLayout(bottom_buttons_hbox);

  setLayout(top_vbox);

  enable_dimensions(_level.drawing_filename.empty());
}

LevelDialog::~LevelDialog()
{
}

void LevelDialog::drawing_filename_button_clicked()
{
  QFileDialog file_dialog(this, "Find Drawing");
  file_dialog.setFileMode(QFileDialog::ExistingFile);
  file_dialog.setNameFilter("*.png");
  if (file_dialog.exec() != QDialog::Accepted)
  {
    if (_drawing_filename_line_edit->text().isEmpty())
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
    if (_drawing_filename_line_edit->text().isEmpty())
      enable_dimensions(true);
    return;
  }
  _drawing_filename_line_edit->setText(
      QDir::current().relativeFilePath(filename));
  enable_dimensions(false);
}

void LevelDialog::ok_button_clicked()
{
  if (!_drawing_filename_line_edit->text().isEmpty())
  {
    // make sure the drawing file exists
    if (!QFileInfo(_drawing_filename_line_edit->text()).exists())
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
  if (_name_line_edit->text().isEmpty()) {
    QMessageBox::critical(
        this,
        "Name must not be empty",
        "Name must not be empty");
    return;
  }
  _level.name = _name_line_edit->text().toStdString();
  _level.elevation = _elevation_line_edit->text().toDouble();
  _level.drawing_filename = _drawing_filename_line_edit->text().toStdString();
  if (_level.drawing_filename.empty())
  {
    _level.x_meters = _x_line_edit->text().toDouble();
    _level.y_meters = _y_line_edit->text().toDouble();
  }
  else
  {
    _level.x_meters = 0.0;
    _level.y_meters = 0.0;
  }
  _level.calculate_scale();
  accept();
}

void LevelDialog::enable_dimensions(const bool enable)
{
  if (enable)
  {
    _x_line_edit->setEnabled(true);
    _y_line_edit->setEnabled(true);
  }
  else
  {
    _x_line_edit->setText("10");
    _y_line_edit->setText("10");
    _x_line_edit->setEnabled(false);
    _y_line_edit->setEnabled(false);
  }
}

void LevelDialog::drawing_filename_line_edited(const QString &text)
{
  enable_dimensions(text.isEmpty());
}
