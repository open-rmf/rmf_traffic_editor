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

#ifndef LAYER_DIALOG_H
#define LAYER_DIALOG_H

#include <QDialog>
#include <QObject>
#include "layer.h"
class QLineEdit;


class LayerDialog : public QDialog
{
  Q_OBJECT

public:
  LayerDialog(QWidget* parent, Layer& _layer, bool edit_mode = true);
  ~LayerDialog();

  void set_center(const double x, const double y);

private:
  Layer& layer;
  bool _edit_mode = true;

  QLineEdit* name_line_edit;
  QLineEdit* filename_line_edit;
  QLineEdit* scale_line_edit;
  QLineEdit* translation_x_line_edit;
  QLineEdit* translation_y_line_edit;
  QLineEdit* rotation_line_edit;

  QPushButton* filename_button;
  QPushButton* center_in_window_button;
  QPushButton* ok_button, * cancel_button;

  void update_layer();

private slots:
  void filename_button_clicked();
  void ok_button_clicked();
  void filename_line_edited(const QString& text);
  void center_in_window_clicked();

signals:
  void redraw();
  void center_layer();
};

#endif
