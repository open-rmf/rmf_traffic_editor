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

#include "layer_table.h"
#include <QtWidgets>

LayerTable::LayerTable()
: TableList(4)
{
  const QStringList labels =
  { "Name", "Active?", "Visible", "Edit"};
  setHorizontalHeaderLabels(labels);
}

LayerTable::~LayerTable()
{
}

void LayerTable::update(BuildingLevel* level)
{
  if (!level)
  {
    layers_table->clearContents();
    return; // let's not crash...
  }

  blockSignals(true);  // otherwise we get tons of callbacks
  setRowCount(2 + level->layers.size());

  layers_table_set_row(0, "Floorplan", true);

  for (size_t i = 0; i < level->layers.size(); i++)
  {
    layers_table_set_row(
      level,
      i + 1,
      QString::fromStdString(level->layers[i].name),
      level->layers[i].visible);
  }

  const int last_row_idx = static_cast<int>(level->layers.size()) + 1;
  // we'll use the last row for the "Add" button
  setCellWidget(last_row_idx, 0, nullptr);
  setCellWidget(last_row_idx, 1, nullptr);
  setCellWidget(last_row_idx, 2, nullptr);
  QPushButton* add_button = new QPushButton("Add...", this);
  setCellWidget(last_row_idx, 3, add_button);
  connect(
    add_button, &QAbstractButton::clicked,
    [=]() { this->layer_add_button_clicked(); });

  blockSignals(false);  // re-enable callbacks
  //sanity_check_layer_table_names(row_idx);
}

void LayerTable::set_row(
  BuildingLevel* level,
  const int row_idx,
  const QString& label,
  const bool checked)
{
  if (!level)
  {
    layers_table->clearContents();
    return; // let's not crash...
  }

  setCellWidget(row_idx, 0, new QLabel(label));

  QCheckBox* active_checkbox = new QCheckBox();
  active_checkbox->setChecked(false);
  setCellWidget(row_idx, 1, active_checkbox);

  QCheckBox* visible_checkbox = new QCheckBox();
  visible_checkbox->setChecked(checked);
  setCellWidget(row_idx, 2, visible_checkbox);

  QPushButton* button = new QPushButton("Edit...", this);
  setCellWidget(row_idx, 3, button);

  connect(
    active_checkbox, &QAbstractButton::clicked,
    [=](bool)
    {
      update_active_layer_checkboxes(row_idx);
      create_scene();
    });
  connect(
    visible_checkbox, &QAbstractButton::clicked,
    [=](bool box_checked)
    {
      if (row_idx == 0)
      {
        level->set_drawing_visible(box_checked);
      }
      else if (row_idx > 0)
      {
        level->layers[row_idx-1].visible = box_checked;
      }
      emit redraw_scene();
    });

  connect(
    button, &QAbstractButton::clicked,
    [=]() { emit edit_button_clicked(row_idx); });
}
