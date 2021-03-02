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

#include <cmath>
#include <fstream>

#include "project.h"
#include "yaml_utils.h"

#include <QFileInfo>
#include <QDir>
#include <QGraphicsItem>

using std::string;
using std::vector;


Project::Project()
{
  for (size_t i = 0; i < rendering_options.show_building_lanes.size(); i++)
    rendering_options.show_building_lanes[i] = true;
}

Project::~Project()
{
}

#if 0
Polygon::EdgeDragPolygon Project::polygon_edge_drag_press(
  const EditorModeId mode,
  const int level_idx,
  const Polygon* polygon,
  const double x,
  const double y)
{
  Polygon::EdgeDragPolygon edp;

  if (level_idx < 0 || level_idx > static_cast<int>(building.levels.size()))
    return edp;// oh no

  if (mode == MODE_BUILDING)
    return building.levels[level_idx].polygon_edge_drag_press(polygon, x, y);
  return edp;
}

void Project::set_selected_containing_polygon(
  const EditorModeId mode,
  const int level_idx,
  const double x,
  const double y)
{
  Level* level = nullptr;
  if (mode == MODE_BUILDING)
    level = &building.levels[level_idx];
  if (level == nullptr)
    return;

  // holes are "higher" in our Z-stack (to make them clickable), so first
  // we need to make a list of all polygons that contain this point.
  vector<Polygon*> containing_polygons;
  for (size_t i = 0; i < level->polygons.size(); i++)
  {
    Polygon& polygon = level->polygons[i];
    QVector<QPointF> polygon_vertices;
    for (const auto& vertex_idx: polygon.vertices)
    {
      const Vertex& v = level->vertices[vertex_idx];
      polygon_vertices.append(QPointF(v.x, v.y));
    }
    QPolygonF qpolygon(polygon_vertices);
    if (qpolygon.containsPoint(QPoint(x, y), Qt::OddEvenFill))
      containing_polygons.push_back(&level->polygons[i]);
  }

  // first search for holes
  for (Polygon* p : containing_polygons)
  {
    if (p->type == Polygon::HOLE)
    {
      p->selected = true;
      return;
    }
  }

  // if we get here, just return the first thing.
  for (Polygon* p : containing_polygons)
  {
    p->selected = true;
    return;
  }
}
#endif

void Project::clear()
{
  building.clear();
  name.clear();
  filename.clear();
}

void Project::clear_scene()
{
  building.clear_scene();
}

void Project::add_lane(
  const int /*level_idx*/,
  const int /*start_idx*/,
  const int /*end_idx*/)
{
  //building.add_lane(level_idx, start_idx, end_idx, traffic_map_idx);
}
