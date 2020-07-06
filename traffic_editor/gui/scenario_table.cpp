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

#include "scenario_dialog.h"
#include "scenario_table.h"
#include <QtWidgets>
using std::unique_ptr;


ScenarioTable::ScenarioTable()
: TableList(2)
{
  const QStringList labels = { "#", "Name", "" };
  setHorizontalHeaderLabels(labels);
}

ScenarioTable::~ScenarioTable()
{
}

void ScenarioTable::update(Project& project)
{
  blockSignals(true);
  setRowCount(1 + project.scenarios.size());
  for (size_t i = 0; i < project.scenarios.size(); i++)
  {
    const Scenario& scenario = *project.scenarios[i];

    QTableWidgetItem* name_item =
      new QTableWidgetItem(QString::fromStdString(scenario.name));

    if (static_cast<int>(i) == project.scenario_idx)
      name_item->setBackground(QBrush(QColor("#e0ffe0")));

    setItem(i, 0, name_item);

    QPushButton* edit_button = new QPushButton("Edit...", this);
    setCellWidget(i, 1, edit_button);
    connect(
      edit_button,
      &QAbstractButton::clicked,
      [this, &project, i]()
      {
        ScenarioDialog dialog(*project.scenarios[i]);
        dialog.exec();
        update(project);
        emit redraw();
      });
  }

  // we'll use the last row for the "Add" button
  const int last_row_idx = static_cast<int>(project.scenarios.size());
  setCellWidget(last_row_idx, 0, nullptr);
  QPushButton* add_button = new QPushButton("Add...", this);
  setCellWidget(last_row_idx, 1, add_button);
  connect(
    add_button, &QAbstractButton::clicked,
    [this, &project]()
    {
      unique_ptr<Scenario> scenario = std::make_unique<Scenario>();
      ScenarioDialog scenario_dialog(*scenario);
      if (scenario_dialog.exec() == QDialog::Accepted)
      {
        project.scenarios.push_back(std::move(scenario));
        update(project);
        emit redraw();
      }
      else
        scenario.release();
    });

  blockSignals(false);
}
