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

#include "zone_dialog.h"
#include "zone_table.h"
#include <QtWidgets>

ZoneTable::ZoneTable()
: TableList(4)
{
  const QStringList labels = { "Name", "Show", "", ""};
  setHorizontalHeaderLabels(labels);
}

ZoneTable::~ZoneTable()
{
}

void ZoneTable::update(Building& building)
{
  blockSignals(true);
  setRowCount(1 + building.zones.size());
  setColumnCount(4);

  for (std::size_t i = 0; i < building.zones.size(); i++)
  {
    QCheckBox* checkbox = new QCheckBox;
    checkbox->setChecked(building.zones[i].show_zone);
    setCellWidget(i, 1, checkbox);
    connect(
      checkbox,
      &QAbstractButton::clicked,
      [this, &building, i](bool box_checked)
      {
        building.zones[i].show_zone = box_checked;
        emit redraw();
      });

    setItem(
      i,
      0,
      new QTableWidgetItem(
        QString::fromStdString(building.zones[i].name)));

    QPushButton* edit_button = new QPushButton("Edit...", this);
    setCellWidget(i, 2, edit_button);

    QPushButton* delete_button = new QPushButton("Delete...", this);
    setCellWidget(i, 3, delete_button);

    connect(
      edit_button,
      &QAbstractButton::clicked,
      [this, &building, i]()
      {
        ZoneDialog* dialog = new ZoneDialog(building.zones[i], building);
        dialog->show();
        dialog->raise();
        dialog->activateWindow();
        connect(
          dialog,
          &ZoneDialog::redraw,
          [this, &building]()
          {
            update(building);
            emit redraw();
          });
      });

    connect(
      delete_button,
      &QAbstractButton::clicked,
      [this, &building, i]()
      {
        if (i < building.zones.size())
        {
          building.zones.erase(building.zones.begin() + i);
          update(building);
          emit redraw();
        }
      });
  }

  // we'll use the last row for the "Add" button
  const int last_row_idx = static_cast<int>(building.zones.size());
  setCellWidget(last_row_idx, 0, nullptr);
  setItem(
    last_row_idx,
    0,
    new QTableWidgetItem(
      QString::fromStdString("")));
  QPushButton* add_button = new QPushButton("Add...", this);
  setCellWidget(last_row_idx, 2, add_button);
  removeCellWidget(last_row_idx, 1);
  connect(
    add_button, &QAbstractButton::clicked,
    [this, &building]()
    {
      Zone zone;
      zone.level = building.levels[0].name;
      zone.elevation = building.levels[0].elevation;
      ZoneDialog zone_dialog(zone, building);
      if (zone_dialog.exec() == QDialog::Accepted)
      {
        building.zones.push_back(zone);
        update(building);
        emit redraw();
      }
    });

  blockSignals(false);
  resizeColumnToContents(2);
  resizeColumnToContents(3);
}
