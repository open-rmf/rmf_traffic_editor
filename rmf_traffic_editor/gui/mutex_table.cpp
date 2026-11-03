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

#include <map>
#include <set>
#include <utility>

#include <QtWidgets>

#include "mutex_table.h"

namespace {

using MutexKey = std::pair<int, std::string>;  // (graph_idx, group name)

struct MutexCounts
{
  int lanes = 0;
  int waypoints = 0;
  std::set<std::string> levels;
};

}  // namespace

MutexTable::MutexTable()
: TableList(4)
{
  const QStringList labels = { "Mutex group", "Graph", "Lanes", "Waypoints" };
  setHorizontalHeaderLabels(labels);
  setSelectionMode(QAbstractItemView::NoSelection);
  setEditTriggers(QAbstractItemView::NoEditTriggers);
}

MutexTable::~MutexTable()
{
}

void MutexTable::update(const Building& building, const RenderingOptions& opts)
{
  std::map<MutexKey, MutexCounts> groups;

  for (const auto& level : building.levels)
  {
    std::map<int, std::set<int>> vertex_graphs;
    for (const auto& edge : level.edges)
    {
      if (edge.type != Edge::LANE)
        continue;
      const int graph_idx = edge.get_graph_idx();
      vertex_graphs[edge.start_idx].insert(graph_idx);
      vertex_graphs[edge.end_idx].insert(graph_idx);

      const std::string group = edge.mutex_group();
      if (group.empty())
        continue;
      MutexCounts& c = groups[{graph_idx, group}];
      c.lanes++;
      c.levels.insert(level.name);
    }

    for (const auto& [vertex_idx, graph_idxs] : vertex_graphs)
    {
      if (vertex_idx < 0 ||
        vertex_idx >= static_cast<int>(level.vertices.size()))
        continue;
      const std::string group = level.vertices[vertex_idx].mutex_group();
      if (group.empty())
        continue;
      for (const int graph_idx : graph_idxs)
      {
        MutexCounts& c = groups[{graph_idx, group}];
        c.waypoints++;
        c.levels.insert(level.name);
      }
    }
  }

  blockSignals(true);
  setRowCount(static_cast<int>(groups.size()));

  int row = 0;
  for (const auto& [key, counts] : groups)
  {
    const bool highlighted = opts.is_mutex_highlighted(key.first, key.second);

    QStringList level_names;
    for (const auto& name : counts.levels)
      level_names << QString::fromStdString(name);
    const QString tooltip =
      QString("Levels: %1\nClick to toggle highlight").arg(
      level_names.join(", "));

    QTableWidgetItem* name_item =
      new QTableWidgetItem(QString::fromStdString(key.second));
    name_item->setData(Qt::UserRole, key.first);

    const QList<QTableWidgetItem*> items = {
      name_item,
      new QTableWidgetItem(QString::number(key.first)),
      new QTableWidgetItem(QString::number(counts.lanes)),
      new QTableWidgetItem(QString::number(counts.waypoints))
    };

    for (int col = 0; col < items.size(); col++)
    {
      items[col]->setToolTip(tooltip);
      if (highlighted)
        items[col]->setBackground(QBrush(QColor("#ffd699")));
      setItem(row, col, items[col]);
    }
    row++;
  }

  blockSignals(false);
}

void MutexTable::toggle_highlight(const int row, RenderingOptions& opts) const
{
  const QTableWidgetItem* name_item = item(row, 0);
  if (!name_item)
    return;

  const int graph_idx = name_item->data(Qt::UserRole).toInt();
  const std::string group = name_item->text().toStdString();

  if (opts.is_mutex_highlighted(graph_idx, group))
  {
    opts.highlight_mutex_graph_idx = -1;
    opts.highlight_mutex_group.clear();
  }
  else
  {
    opts.highlight_mutex_graph_idx = graph_idx;
    opts.highlight_mutex_group = group;
  }
}

void MutexTable::set_highlight(const int row, RenderingOptions& opts) const
{
  const QTableWidgetItem* name_item = item(row, 0);
  if (!name_item)
    return;

  opts.highlight_mutex_graph_idx = name_item->data(Qt::UserRole).toInt();
  opts.highlight_mutex_group = name_item->text().toStdString();
}

void MutexTable::mousePressEvent(QMouseEvent* event)
{
  _row_before_press = currentRow();
  TableList::mousePressEvent(event);
}
