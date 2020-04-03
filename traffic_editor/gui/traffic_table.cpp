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
using std::shared_ptr;


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

void TrafficTable::update(std::shared_ptr<Project> project)
{
  blockSignals(true);
  setRowCount(1 + project->traffic_maps.size());

  for (size_t i = 0; i < project->traffic_maps.size(); i++)
  {
    const TrafficMap& traffic_map = project->traffic_maps[i];

    QCheckBox *checkbox = new QCheckBox;
    checkbox->setChecked(traffic_map.visible);
    setCellWidget(i, 0, checkbox);
    connect(
        checkbox,
        &QAbstractButton::clicked,
        [this, project, i](bool box_checked)
        {
          project->traffic_maps[i].visible = box_checked;
          emit redraw();
        });

    QTableWidgetItem *name_item =
        new QTableWidgetItem(QString::fromStdString(traffic_map.name));

    if (static_cast<int>(i) == project->traffic_map_idx)
      name_item->setBackground(QBrush(QColor("#e0ffe0")));

    setItem(i, 1, name_item);

    QPushButton *edit_button = new QPushButton("Edit...", this);
    setCellWidget(i, 2, edit_button);
    connect(
        edit_button,
        &QAbstractButton::clicked,
        [this, project, i]()
        {
          TrafficMapDialog dialog(project->traffic_maps[i]);
          dialog.exec();
          update(project);
          emit redraw();
        });
  }

  // we'll use the last row for the "Add" button
  const int last_row_idx = static_cast<int>(project->scenarios.size());
  setCellWidget(last_row_idx, 0, nullptr);
  setCellWidget(last_row_idx, 1, nullptr);
  QPushButton *add_button = new QPushButton("Add...", this);
  setCellWidget(last_row_idx, 2, add_button);
  connect(
      add_button,
      &QAbstractButton::clicked,
      [this, project]()
      {
        TrafficMap traffic_map;
        TrafficMapDialog dialog(traffic_map);
        if (dialog.exec() == QDialog::Accepted)
        {
          project->traffic_maps.push_back(traffic_map);
          update(project);
          emit redraw();
        }
      });

  blockSignals(false);
}
