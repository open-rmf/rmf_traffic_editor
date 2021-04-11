/*
 * Copyright (C) 2019-2021 Open Source Robotics Foundation
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
    { "Name", "Color", "Visible", "Edit"};
  setHorizontalHeaderLabels(labels);
}

LayerTable::~LayerTable()
{
}

void LayerTable::update(Building& building, const int level_idx)
{
  if (level_idx >= static_cast<int>(building.levels.size()))
  {
    clearContents();
    return;
  }

  Level& level = building.levels[level_idx];

  blockSignals(true);  // otherwise we get tons of callbacks
  setRowCount(2 + level.layers.size());

  set_row(level, 0, "Floorplan", QColor::fromRgbF(0, 0, 0, 1), true);

  for (size_t i = 0; i < level.layers.size(); i++)
  {
    set_row(
      level,
      i + 1,
      QString::fromStdString(level.layers[i].name),
      row_color(i + 1),
      level.layers[i].visible);
  }

  const int last_row_idx = static_cast<int>(level.layers.size()) + 1;
  // we'll use the last row for the "Add" button
  setCellWidget(last_row_idx, 0, nullptr);
  setCellWidget(last_row_idx, 1, nullptr);
  setCellWidget(last_row_idx, 2, nullptr);
  QPushButton* add_button = new QPushButton("Add...", this);
  setCellWidget(last_row_idx, 3, add_button);
  connect(
    add_button,
    &QAbstractButton::clicked,
    [=]() { emit add_button_clicked(); });

  blockSignals(false);  // re-enable callbacks
}

void LayerTable::set_row(
  Level& level,
  const int row_idx,
  const QString& label,
  const QColor& color,
  const bool checked)
{
  QTableWidgetItem* name_item = new QTableWidgetItem(label);
  setItem(row_idx, 0, name_item);

  QTableWidgetItem* color_item = new QTableWidgetItem(QString());
  color_item->setBackground(QBrush(color));
  setItem(row_idx, 1, color_item);

  QCheckBox* visible_checkbox = new QCheckBox();
  visible_checkbox->setChecked(checked);
  setCellWidget(row_idx, 2, visible_checkbox);

  QPushButton* button = new QPushButton("Edit...", this);
  setCellWidget(row_idx, 3, button);

  connect(
    visible_checkbox,
    &QAbstractButton::clicked,
    [&](bool box_checked)
    {
      if (row_idx == 0)
      {
        level.set_drawing_visible(box_checked);
      }
      else if (row_idx > 0)
      {
        level.layers[row_idx-1].visible = box_checked;
      }
      emit redraw_scene();
    });

  connect(
    button,
    &QAbstractButton::clicked,
    [=]() { emit edit_button_clicked(row_idx); });
}

QColor LayerTable::row_color(const int row_idx)
{
  switch (row_idx)
  {
    case 0:
    default: return QColor::fromRgbF(0, 0, 0, 1.0);
    case 1:  return QColor::fromRgbF(1, 0, 0, 1.0);
    case 2:  return QColor::fromRgbF(0, 1, 0, 1.0);
    case 3:  return QColor::fromRgbF(0, 0, 1, 1.0);
    case 4:  return QColor::fromRgbF(1, 1, 0, 1.0);
    case 5:  return QColor::fromRgbF(0, 1, 1, 1.0);
    case 6:  return QColor::fromRgbF(1, 0, 1, 1.0);
  }
}
