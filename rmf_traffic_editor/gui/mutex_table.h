/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#ifndef MUTEX_TABLE_H
#define MUTEX_TABLE_H

#include "building.h"
#include "rendering_options.h"
#include "table_list.h"

class MutexTable : public TableList
{
  Q_OBJECT
public:
  MutexTable();
  ~MutexTable();

  void update(const Building& building, const RenderingOptions& opts);

  /// Toggle highlighting of the (graph, mutex group) shown in this row
  void toggle_highlight(const int row, RenderingOptions& opts) const;

  void set_highlight(const int row, RenderingOptions& opts) const;

  int row_before_press() const { return _row_before_press; }

protected:
  void mousePressEvent(QMouseEvent* event) override;

private:
  int _row_before_press = -1;
};

#endif
