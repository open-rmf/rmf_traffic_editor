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

#include "lift_table.h"
#include "lift_dialog.h"
#include <QtWidgets>

LiftTable::LiftTable()
: TableList()
{
}

LiftTable::~LiftTable()
{
}

void LiftTable::update(Map& map)
{
  blockSignals(true);
  setRowCount(1 + map.lifts.size());
  for (size_t i = 0; i < map.lifts.size(); i++)
  {
    setItem(
        i,
        0,
        new QTableWidgetItem(
            QString::fromStdString(map.lifts[i].name)));

    QPushButton *edit_button = new QPushButton("Edit...", this);
    setCellWidget(i, 1, edit_button);

    connect(
        edit_button,
        &QAbstractButton::clicked,
        [this, &map, i]() {
          /*
          LiftDialog lift_dialog(map.lifts[i], map);
          lift_dialog.exec();
          update(map);
          emit redraw();
          */
          LiftDialog *dialog = new LiftDialog(map.lifts[i], map);
          dialog->show();
          dialog->raise();
          dialog->activateWindow();
          connect(
              dialog,
              &LiftDialog::redraw,
              [this]() { emit redraw(); });
        });
  }

  // we'll use the last row for the "Add" button
  const int last_row_idx = static_cast<int>(map.lifts.size());
  setCellWidget(last_row_idx, 0, nullptr);
  QPushButton *add_button = new QPushButton("Add...", this);
  setCellWidget(last_row_idx, 1, add_button);
  connect(
      add_button, &QAbstractButton::clicked,
      [this, &map]() {
        Lift lift;
        LiftDialog lift_dialog(lift, map);
        if (lift_dialog.exec() == QDialog::Accepted)
        {
          map.lifts.push_back(lift);
          update(map);
          emit redraw();
        }
      });

  blockSignals(false);
}
