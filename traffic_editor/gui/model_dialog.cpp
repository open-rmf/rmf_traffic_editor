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

#include "model_dialog.h"
#include <QtWidgets>
using std::vector;


ModelDialog::ModelDialog(
    QWidget *parent,
    Model& model,
    const vector<EditorModel>& editor_models)
: QDialog(parent),
  _model(model),
  _editor_models(editor_models)
{
  QHBoxLayout *bottom_buttons_layout = new QHBoxLayout;
  ok_button = new QPushButton("OK", this);  // first button = [enter] button
  bottom_buttons_layout->addWidget(ok_button);
  connect(
      ok_button, &QAbstractButton::clicked,
      this, &ModelDialog::ok_button_clicked);

  cancel_button = new QPushButton("Cancel", this);
  bottom_buttons_layout->addWidget(cancel_button);
  connect(
    cancel_button, &QAbstractButton::clicked,
    this, &QDialog::reject);

  QVBoxLayout *model_name_vbox_layout = new QVBoxLayout;
  model_name_vbox_layout->addWidget(new QLabel("Name:"));

  model_name_line_edit =
      new QLineEdit(QString::fromStdString(model.model_name), this);
  model_name_vbox_layout->addWidget(model_name_line_edit);
  connect(
      model_name_line_edit,
      &QLineEdit::textEdited,
      this,
      &ModelDialog::model_name_line_edited);

  model_name_list_widget = new QListWidget;
  model_name_vbox_layout->addWidget(model_name_list_widget);

  QHBoxLayout *top_hbox_layout = new QHBoxLayout;
  top_hbox_layout->addLayout(model_name_vbox_layout);

  QVBoxLayout *vbox_layout = new QVBoxLayout;
  vbox_layout->addLayout(top_hbox_layout);
  // todo: some sort of separator (?)
  vbox_layout->addLayout(bottom_buttons_layout);

  setLayout(vbox_layout);

  for (const auto& em : _editor_models)
    model_name_list_widget->addItem(QString::fromStdString(em.name));
}

ModelDialog::~ModelDialog()
{
}

void ModelDialog::ok_button_clicked()
{
  if (model_name_line_edit->text().isEmpty())
  {
    QMessageBox::critical(this, "Error", "Model name missing");
    return;
  }

  accept();
}

void ModelDialog::model_name_line_edited(const QString &/*text*/)
{
  // todo: render on parent if file exists?
}
