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

#include "level_table.h"
#include "level_dialog.h"
#include <QtWidgets>

LevelTable::LevelTable()
: TableList(6)
{
  const QStringList labels =
      { "Name", "Scale", "X", "Y", "Z", "" };
  setHorizontalHeaderLabels(labels);
}

LevelTable::~LevelTable()
{
}

void LevelTable::update(Map& map)
{
  blockSignals(true);  // avoid tons of callbacks

  setRowCount(1 + map.levels.size());
  const int reference_level_idx = map.get_reference_level_idx();

  for (size_t i = 0; i < map.levels.size(); i++)
  {
    QTableWidgetItem *name_item = 
        new QTableWidgetItem(QString::fromStdString(map.levels[i].name));
    setItem(i, 0, name_item);

    if (static_cast<int>(i) == reference_level_idx)
      name_item->setBackground(QBrush(QColor("#e0ffe0")));

    setItem(
        i,
        1,
        new QTableWidgetItem(
            QString::number(
                map.levels[i].drawing_meters_per_pixel,
                'f',
                4)));

    Map::Transform t = map.get_transform(reference_level_idx, i);

    setItem(i, 2, new QTableWidgetItem(QString::number(t.dx, 'f', 1)));
    setItem(i, 3, new QTableWidgetItem(QString::number(t.dy, 'f', 1)));

    setItem(
        i,
        4,
        new QTableWidgetItem(
            QString::number(map.levels[i].elevation, 'f', 1)));

    QPushButton *edit_button = new QPushButton("Edit...", this);
    setCellWidget(i, 5, edit_button);
    edit_button->setStyleSheet("QTableWidgetItem { background-color: red; }");

    connect(
        edit_button, &QAbstractButton::clicked,
        [this, &map, i]()
        {
          LevelDialog level_dialog(map.levels[i]);
          if (level_dialog.exec() == QDialog::Accepted)
          {
            setWindowModified(true);  // not sure why, but this doesn't work
          }
          update(map);
        });
  }

  const int last_row_idx = static_cast<int>(map.levels.size());
  // we'll use the last row for the "Add" button
  setCellWidget(last_row_idx, 0, nullptr);
  QPushButton *add_button = new QPushButton("Add...", this);
  setCellWidget(last_row_idx, 5, add_button);
  connect(
      add_button, &QAbstractButton::clicked,
      [this, &map]()
      {
        Level level;
        LevelDialog level_dialog(level);
        if (level_dialog.exec() == QDialog::Accepted)
        {
          map.add_level(level);
          setWindowModified(true);
          emit redraw_scene();
        }
      });

  blockSignals(false);
}
