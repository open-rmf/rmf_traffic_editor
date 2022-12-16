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
  const QStringList labels = { "Name", "Color", "Visible", "Edit"};
  setHorizontalHeaderLabels(labels);
}

LayerTable::~LayerTable()
{
}

void LayerTable::update(
  Building& building,
  const int level_idx,
  const int layer_idx)
{
  if (level_idx >= static_cast<int>(building.levels.size()))
  {
    clearContents();
    return;
  }

  Level& level = building.levels[level_idx];

  blockSignals(true);  // otherwise we get tons of callbacks
  setRowCount(2 + level.layers.size());

  set_row(
    level,
    0,
    "Floorplan",
    QColor::fromRgbF(0, 0, 0, 1),
    level.get_drawing_visible(),
    layer_idx == 0);

  for (std::size_t i = 0; i < level.layers.size(); i++)
  {
    set_row(
      level,
      i + 1,
      QString::fromStdString(level.layers[i].name),
      level.layers[i].color,
      level.layers[i].visible,
      layer_idx == static_cast<int>(i + 1));
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
  const bool checked,
  const bool is_active_layer)
{
  QTableWidgetItem* name_item = new QTableWidgetItem(label);
  if (is_active_layer)
  {
    QFont font;
    font.setBold(true);
    name_item->setFont(font);
  }
  setItem(row_idx, 0, name_item);

  QPushButton* color_button = new QPushButton("", this);

  color_button->setStyleSheet(
    QString::asprintf(
      "background-color: rgb(%d, %d, %d)",
      color.red(),
      color.green(),
      color.blue()));

  setCellWidget(row_idx, 1, color_button);

  if (row_idx > 0)
  {
    connect(
      color_button,
      &QAbstractButton::clicked,
      [this, &level, color_button, row_idx]()
      {
        QColor selected_color = QColorDialog::getColor(
          level.layers[row_idx - 1].color);
        if (selected_color.isValid())
        {
          selected_color.setAlphaF(0.5);
          level.layers[row_idx - 1].color = selected_color;
          level.layers[row_idx - 1].colorize_image();
          emit redraw_scene();
        }
      }
    );
  }

  QCheckBox* visible_checkbox = new QCheckBox();
  visible_checkbox->setChecked(checked);
  setCellWidget(row_idx, 2, visible_checkbox);

  QPushButton* button = new QPushButton("Edit...", this);
  setCellWidget(row_idx, 3, button);

  connect(
    visible_checkbox,
    &QAbstractButton::clicked,
    [this, &level, row_idx](bool box_checked)
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
