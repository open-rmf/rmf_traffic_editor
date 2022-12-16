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

#include "traffic_table.h"
#include "traffic_map_dialog.h"
#include <QtWidgets>


TrafficTable::TrafficTable()
: TableList(3)
{
  const QStringList labels = { "Show", "Name", "" };
  setHorizontalHeaderLabels(labels);
  horizontalHeader()->setSectionResizeMode(
    0, QHeaderView::ResizeToContents);
  horizontalHeader()->setSectionResizeMode(
    1, QHeaderView::Stretch);
}

TrafficTable::~TrafficTable()
{
}

void TrafficTable::update(RenderingOptions& opts)
{
  blockSignals(true);

  const std::size_t num_lanes = opts.show_building_lanes.size();
  setRowCount(num_lanes);

  for (std::size_t i = 0; i < num_lanes; i++)
  {
    QCheckBox* checkbox = new QCheckBox;
    checkbox->setChecked(opts.show_building_lanes[i]);
    setCellWidget(i, 0, checkbox);
    connect(
      checkbox,
      &QAbstractButton::clicked,
      [this, &opts, i](bool box_checked)
      {
        opts.show_building_lanes[i] = box_checked;
        emit redraw();
      });


    QTableWidgetItem* name_item =
      new QTableWidgetItem(QString("Graph %1").arg(i));

    if (static_cast<int>(i) == opts.active_traffic_map_idx)
      name_item->setBackground(QBrush(QColor("#e0ffe0")));

    setItem(i, 1, name_item);
  }

  blockSignals(false);
}
