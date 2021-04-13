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

#ifndef LAYER_TABLE_H
#define LAYER_TABLE_H

#include <QTableWidget>

#include "building.h"
#include "table_list.h"
#include "level.h"

class LayerTable : public TableList
{
  Q_OBJECT

public:
  LayerTable();
  ~LayerTable();

  void update(Building& building, const int level_idx, const int layer_idx);

  void set_row(
    Level& level,
    const int row_idx,
    const QString& label,
    const QColor& color,
    const bool checked,
    const bool is_active_layer);

signals:
  void redraw_scene();
  void add_button_clicked();
  void edit_button_clicked(const int row_idx);
};

#endif
