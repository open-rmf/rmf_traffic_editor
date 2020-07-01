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

#include "layer_dialog.h"
#include <QtWidgets>


LayerDialog::LayerDialog(QWidget* parent, Layer& _layer, bool edit_mode)
: QDialog(parent),
  layer(_layer),
  _edit_mode(edit_mode)
{
  setWindowTitle("Layer Properties");
  QHBoxLayout* bottom_buttons_layout = new QHBoxLayout;
  ok_button = new QPushButton("OK", this);  // first button = [enter] button
  bottom_buttons_layout->addWidget(ok_button);
  connect(
    ok_button, &QAbstractButton::clicked,
    this, &LayerDialog::ok_button_clicked);

  // When using this dialog in "edit mode," it is modeless and instantly
  // updating, so there is no "cancel" function.
  if (!edit_mode)
  {
    cancel_button = new QPushButton("Cancel", this);
    bottom_buttons_layout->addWidget(cancel_button);
    connect(
      cancel_button, &QAbstractButton::clicked,
      this, &QDialog::reject);
  }

  QHBoxLayout* name_hbox_layout = new QHBoxLayout;
  name_line_edit = new QLineEdit(QString::fromStdString(layer.name), this);
  name_hbox_layout->addWidget(new QLabel("name:"));
  name_hbox_layout->addWidget(name_line_edit);

  QHBoxLayout* filename_layout = new QHBoxLayout;
  filename_line_edit = new QLineEdit(
    QString::fromStdString(layer.filename), this);
  filename_button = new QPushButton("Find...", this);
  filename_layout->addWidget(new QLabel("image:"));
  filename_layout->addWidget(filename_line_edit);
  filename_layout->addWidget(filename_button);
  connect(
    filename_button, &QAbstractButton::clicked,
    this, &LayerDialog::filename_button_clicked);
  connect(
    filename_line_edit,
    &QLineEdit::textEdited,
    this,
    &LayerDialog::filename_line_edited);

  QHBoxLayout* scale_hbox_layout = new QHBoxLayout;
  scale_hbox_layout->addWidget(new QLabel("Meters per pixel:"));
  scale_line_edit = new QLineEdit(
    QString::number(layer.meters_per_pixel),
    this);
  scale_hbox_layout->addWidget(scale_line_edit);

  QHBoxLayout* translation_x_hbox_layout = new QHBoxLayout;
  translation_x_hbox_layout->addWidget(new QLabel("X translation (meters):"));
  translation_x_line_edit = new QLineEdit(
    QString::number(layer.translation_x),
    this);
  translation_x_hbox_layout->addWidget(translation_x_line_edit);

  QHBoxLayout* translation_y_hbox_layout = new QHBoxLayout;
  translation_y_hbox_layout->addWidget(new QLabel("Y translation (meters):"));
  translation_y_line_edit = new QLineEdit(
    QString::number(layer.translation_y),
    this);
  translation_y_hbox_layout->addWidget(translation_y_line_edit);

  QHBoxLayout* rotation_hbox_layout = new QHBoxLayout;
  rotation_hbox_layout->addWidget(new QLabel("Rotation (radians):"));
  rotation_line_edit = new QLineEdit(
    QString::number(layer.rotation),
    this);
  rotation_hbox_layout->addWidget(rotation_line_edit);

  QVBoxLayout* vbox_layout = new QVBoxLayout;
  vbox_layout->addLayout(name_hbox_layout);
  vbox_layout->addLayout(filename_layout);
  vbox_layout->addLayout(scale_hbox_layout);
  vbox_layout->addLayout(translation_x_hbox_layout);
  vbox_layout->addLayout(translation_y_hbox_layout);
  vbox_layout->addLayout(rotation_hbox_layout);
  // todo: some sort of separator (?)
  vbox_layout->addLayout(bottom_buttons_layout);

  connect(
    filename_line_edit,
    &QLineEdit::textEdited,
    this,
    &LayerDialog::update_layer);

  connect(
    scale_line_edit,
    &QLineEdit::textEdited,
    this,
    &LayerDialog::update_layer);

  connect(
    translation_x_line_edit,
    &QLineEdit::textEdited,
    this,
    &LayerDialog::update_layer);

  connect(
    translation_y_line_edit,
    &QLineEdit::textEdited,
    this,
    &LayerDialog::update_layer);

  connect(
    rotation_line_edit,
    &QLineEdit::textEdited,
    this,
    &LayerDialog::update_layer);

  setLayout(vbox_layout);
}

LayerDialog::~LayerDialog()
{
}

void LayerDialog::filename_button_clicked()
{
  QFileDialog file_dialog(this, "Find Image");
  file_dialog.setFileMode(QFileDialog::ExistingFile);
  file_dialog.setNameFilter("*.png");
  if (file_dialog.exec() != QDialog::Accepted)
  {
    return;  // user clicked 'cancel'
  }
  const QString filename = file_dialog.selectedFiles().first();
  if (!QFileInfo(filename).exists())
  {
    QMessageBox::critical(
      this,
      "Image file does not exist",
      "File does not exist.");
    return;
  }
  filename_line_edit->setText(
    QDir::current().relativeFilePath(filename));
}

void LayerDialog::ok_button_clicked()
{
  if (!filename_line_edit->text().isEmpty())
  {
    // make sure the drawing file exists
    if (!QFileInfo(filename_line_edit->text()).exists())
    {
      QMessageBox::critical(
        this,
        "Image file must exist",
        "Image file must exist");
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
  update_layer();

  accept();
}

void LayerDialog::filename_line_edited(const QString& /*text*/)
{
  // todo: render on parent if file exists?
}

void LayerDialog::update_layer()
{
  layer.name = name_line_edit->text().toStdString();
  layer.filename = filename_line_edit->text().toStdString();
  layer.rotation = rotation_line_edit->text().toDouble();
  layer.translation_x = translation_x_line_edit->text().toDouble();
  layer.translation_y = translation_y_line_edit->text().toDouble();
  layer.meters_per_pixel = scale_line_edit->text().toDouble();
  emit redraw();
}
