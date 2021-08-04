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
#include <algorithm>
#include <QtWidgets>
using std::vector;
using std::string;


ModelDialog::ModelDialog(
  QWidget* parent,
  Model& model,
  const vector<EditorModel>& editor_models)
: QDialog(parent),
  _model(model),
  _editor_models(editor_models)
{
  setWindowTitle("Model Properties");
  QHBoxLayout* bottom_buttons_layout = new QHBoxLayout;
  _ok_button = new QPushButton("OK", this);  // first button = [enter] button
  bottom_buttons_layout->addWidget(_ok_button);
  connect(
    _ok_button, &QAbstractButton::clicked,
    this, &ModelDialog::ok_button_clicked);

  _cancel_button = new QPushButton("Cancel", this);
  bottom_buttons_layout->addWidget(_cancel_button);
  connect(
    _cancel_button, &QAbstractButton::clicked,
    this, &QDialog::reject);

  QVBoxLayout* model_name_vbox_layout = new QVBoxLayout;
  model_name_vbox_layout->addWidget(new QLabel("Name:"));

  _model_name_line_edit =
    new QLineEdit(QString::fromStdString(_model.model_name), this);
  model_name_vbox_layout->addWidget(_model_name_line_edit);
  connect(
    _model_name_line_edit,
    &QLineEdit::textEdited,
    this,
    &ModelDialog::model_name_line_edited);

  _model_name_list_widget = new QListWidget;
  model_name_vbox_layout->addWidget(_model_name_list_widget);
  connect(
    _model_name_list_widget,
    &QListWidget::currentRowChanged,
    this,
    &ModelDialog::model_name_list_widget_changed);

  _model_preview_label = new QLabel;
  _model_preview_label->setMinimumSize(600, 600);
  _model_preview_label->setSizePolicy(
    QSizePolicy::MinimumExpanding,
    QSizePolicy::Expanding);

  QHBoxLayout* top_hbox_layout = new QHBoxLayout;
  top_hbox_layout->addLayout(model_name_vbox_layout);
  top_hbox_layout->addWidget(_model_preview_label);

  QVBoxLayout* vbox_layout = new QVBoxLayout;
  vbox_layout->addLayout(top_hbox_layout);
  // todo: some sort of separator (?)
  vbox_layout->addLayout(bottom_buttons_layout);

  setLayout(vbox_layout);

  for (const auto& em : _editor_models)
  {
    string::size_type token_start = em.name.find_first_of('/');
    string token_string = em.name;
    if (token_start != std::string::npos && em.name.size() > token_start)
      token_string = em.name.substr(token_start + 1);

    for (std::size_t i = 0; i < token_string.size(); i++)
      token_string[i] = tolower(token_string[i]);

    sorted_names.push_back(std::make_pair(token_string, em.name));
  }

  std::sort(
    sorted_names.begin(),
    sorted_names.end(),
    [](std::pair<string, string> a, std::pair<string, string> b)
    {
      return a.first < b.first;
    });

  for (const auto& sorted_name : sorted_names)
  {
    _model_name_list_widget->addItem(
      QString::fromStdString(sorted_name.second));
  }
  _model_name_list_widget->setMinimumWidth(
    _model_name_list_widget->sizeHintForColumn(0) + 30);

  if (!_editor_models.empty())
    _model_name_list_widget->setCurrentItem(
      _model_name_list_widget->item(0));

  _model_name_line_edit->setFocus(Qt::OtherFocusReason);
}

ModelDialog::~ModelDialog()
{
}

void ModelDialog::ok_button_clicked()
{
  if (_model.model_name.empty())  // _model_name_line_edit->text().isEmpty())
  {
    QMessageBox::critical(this, "Error", "Model name missing");
    return;
  }

  accept();
}

void ModelDialog::model_name_line_edited(const QString& text)
{
  // todo: render on parent if file exists?
  if (_model_name_list_widget->count() == 0)
  {
    qWarning("model name list widget is empty :(");
    return;  // nothing to do; there is no available model list
  }

  // see if we can auto-complete with anything in the list box
  // scroll the list box to the first thing
  const std::string user_text_lower(text.toLower().toStdString());
  // could become super fancy but for now let's just do linear search...
  std::size_t closest_idx = 0;
  for (std::size_t i = 0; i < sorted_names.size(); i++)
  {
    if (user_text_lower <= sorted_names[i].first)
    {
      closest_idx = i;
      break;
    }
  }
  QListWidgetItem* item = _model_name_list_widget->item(closest_idx);
  _model_name_list_widget->setCurrentItem(item);
  _model_name_list_widget->scrollToItem(
    item,
    QAbstractItemView::PositionAtTop);
}

void ModelDialog::model_name_list_widget_changed(int row)
{
  _model.model_name = sorted_names[row].second;

  // look up the _editor_model instance that has this name
  for (auto& em : _editor_models)
  {
    if (em.name == _model.model_name)
    {
      const QPixmap& model_pixmap = em.get_pixmap();
      if (model_pixmap.isNull())
        return;// we don't have a pixmap to draw :(
      // scale the pixmap so it fits within the currently allotted space
      const int w = _model_preview_label->width();
      const int h = _model_preview_label->height();
      _model_preview_label->setPixmap(
        model_pixmap.scaled(w, h, Qt::KeepAspectRatio));
      break;
    }
  }
}
