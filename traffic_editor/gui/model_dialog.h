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

#ifndef MODEL_DIALOG_H
#define MODEL_DIALOG_H

#include <QDialog>
#include <QObject>
#include "traffic_editor/model.h"
#include "traffic_editor/editor_model.h"
#include <vector>
class QLineEdit;
class QListWidget;
class QLabel;


class ModelDialog : public QDialog
{
  Q_OBJECT

public:
  ModelDialog(
    QWidget* parent,
    Model& model,
    const std::vector<EditorModel>& editor_models);
  ~ModelDialog();

private:
  Model& _model;
  std::vector<EditorModel> _editor_models;

  QLineEdit* _model_name_line_edit;
  QListWidget* _model_name_list_widget;
  QLabel* _model_preview_label;

  QPushButton* _ok_button, * _cancel_button;

private slots:
  void ok_button_clicked();
  void model_name_line_edited(const QString& text);
  void model_name_list_widget_changed(int row);
};

#endif
