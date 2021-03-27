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

#include "table_list.h"
#include <QtWidgets>

TableList::TableList(const int num_cols)
{
  const char* style =
    "QTableWidget { background-color: #e0e0e0; color: black; } "
    "QHeaderView::section { color: black; } "
    "QLineEdit { background:white; } "
    "QCheckBox { padding-left: 5px; background-color: #e0e0e0; } "
    "QPushButton { margin: 5px; background-color: #c0c0c0; border: 1px solid black; } "
    "QPushButton:pressed { background-color: #808080; }";
  setStyleSheet(style);
  setColumnCount(num_cols);
  setMinimumSize(400, 200);

  verticalHeader()->setVisible(false);
  verticalHeader()->setSectionResizeMode(
    QHeaderView::ResizeToContents);

  horizontalHeader()->setVisible(true);
  horizontalHeader()->setDefaultAlignment(Qt::AlignLeft);
  horizontalHeader()->setSectionResizeMode(
    0, QHeaderView::Stretch);

  for (int col = 1; col < num_cols; col++)
    horizontalHeader()->setSectionResizeMode(
      col,
      QHeaderView::ResizeToContents);

  setAutoFillBackground(true);

  setSizePolicy(
    QSizePolicy::Fixed,
    QSizePolicy::MinimumExpanding);
}

TableList::~TableList()
{
}
