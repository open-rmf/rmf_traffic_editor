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

#include "lift_dialog.h"
#include "lift_table.h"
#include <QtWidgets>

LiftTable::LiftTable()
: TableList()
{
  const QStringList labels = { "Name", "" };
  setHorizontalHeaderLabels(labels);
}

LiftTable::~LiftTable()
{
}

void LiftTable::update(Building& building)
{
  blockSignals(true);
  setRowCount(1 + building.lifts.size());
  for (size_t i = 0; i < building.lifts.size(); i++)
  {
    setItem(
      i,
      0,
      new QTableWidgetItem(
        QString::fromStdString(building.lifts[i].name)));

    QPushButton* edit_button = new QPushButton("Edit...", this);
    setCellWidget(i, 1, edit_button);

    connect(
      edit_button,
      &QAbstractButton::clicked,
      [this, &building, i]()
      {
        /*
        LiftDialog lift_dialog(building.lifts[i], building);
        lift_dialog.exec();
        update(building);
        emit redraw();
        */
        LiftDialog* dialog = new LiftDialog(building.lifts[i], building);
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
  const int last_row_idx = static_cast<int>(building.lifts.size());
  setCellWidget(last_row_idx, 0, nullptr);
  setItem(
    last_row_idx,
    0,
    new QTableWidgetItem(
      QString::fromStdString("")));
  QPushButton* add_button = new QPushButton("Add...", this);
  setCellWidget(last_row_idx, 1, add_button);
  connect(
    add_button, &QAbstractButton::clicked,
    [this, &building]()
    {
      Lift lift;
      LiftDialog lift_dialog(lift, building);
      if (lift_dialog.exec() == QDialog::Accepted)
      {
        building.lifts.push_back(lift);
        update(building);
        emit redraw();
      }
    });

  blockSignals(false);
}
