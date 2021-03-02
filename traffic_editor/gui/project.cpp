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

bool Project::export_correspondence_points(
  int level_index,
  const std::string& dest_filename) const
{
  return building.export_level_correspondence_points(
    level_index,
    dest_filename);
}

void Project::draw(
  QGraphicsScene* scene,
  const int level_idx,
  std::vector<EditorModel>& editor_models)
{
  if (building.levels.empty())
  {
    printf("nothing to draw!\n");
    return;
  }

  building.levels[level_idx].draw(scene, editor_models, rendering_options);
  building.draw_lifts(scene, level_idx);
}

void Project::clear_selection(const int level_idx)
{
  if (building.levels.empty())
    return;
  building.levels[level_idx].clear_selection();
}

bool Project::can_delete_current_selection(const int level_idx)
{
  if (level_idx >= static_cast<int>(building.levels.size()))
    return false;
  return building.levels[level_idx].can_delete_current_selection();
}

bool Project::delete_selected(const int level_idx)
{
  if (building.levels.empty())
    return false;
  if (!building.delete_selected(level_idx))
    return false;
  return true;
}

void Project::get_selected_items(
  const int level_idx,
  std::vector<BuildingLevel::SelectedItem>& selected)
{
  building.levels[level_idx].get_selected_items(selected);
}

void Project::mouse_select_press(
  const EditorModeId mode,
  const int level_idx,
  const int /*layer_idx*/,
  const double /*x*/,
  const double /*y*/,
  QGraphicsItem* /*graphics_item*/)
{
  clear_selection(level_idx);
#if 0
  const NearestItem ni = nearest_items(mode, level_idx, layer_idx, x, y);

  const double vertex_dist_thresh =
    building.levels[level_idx].vertex_radius /
    building.levels[level_idx].drawing_meters_per_pixel;

  if (mode == MODE_BUILDING)
  {
    // todo: use QGraphics stuff to see if we clicked a model pixmap...
    const double model_dist_thresh = 0.5 /
      building.levels[level_idx].drawing_meters_per_pixel;

    if (rendering_options.show_models &&
      ni.model_idx >= 0 &&
      ni.model_dist < model_dist_thresh)
      building.levels[level_idx].models[ni.model_idx].selected = true;
    else if (ni.vertex_idx >= 0 && ni.vertex_dist < vertex_dist_thresh)
      building.levels[level_idx].vertices[ni.vertex_idx].selected = true;
    else if (ni.fiducial_idx >= 0 && ni.fiducial_dist < 10.0)
      building.levels[level_idx].fiducials[ni.fiducial_idx].selected = true;
    else
    {
      // use the QGraphics stuff to see if it's an edge segment or polygon
      if (graphics_item)
      {
        switch (graphics_item->type())
        {
          case QGraphicsLineItem::Type:
            set_selected_line_item(
              level_idx,
              qgraphicsitem_cast<QGraphicsLineItem*>(graphics_item),
              mode);
            break;

          case QGraphicsPolygonItem::Type:
            set_selected_containing_polygon(mode, level_idx, x, y);
            break;

          default:
            printf("clicked unhandled type: %d\n",
              static_cast<int>(graphics_item->type()));
            break;
        }
      }
    }
  }
  else if (mode == MODE_TRAFFIC || mode == MODE_CROWD_SIM)
  {
    // todo: keep traffic-map vertices separate from building vertices
    // for now, they're using the same vertex list.

    if (ni.vertex_idx >= 0 && ni.vertex_dist < vertex_dist_thresh)
      building.levels[level_idx].vertices[ni.vertex_idx].selected = true;
    else
    {
      // use the QGraphics stuff to see if it's an edge segment or polygon
      if (graphics_item)
      {
        switch (graphics_item->type())
        {
          case QGraphicsLineItem::Type:
            set_selected_line_item(
              level_idx,
              qgraphicsitem_cast<QGraphicsLineItem*>(graphics_item),
              mode);
            break;

          default:
            printf("clicked unhandled type: %d\n",
              static_cast<int>(graphics_item->type()));
            break;
        }
      }
    }
  }
#endif
}

void Project::set_selected_line_item(
  const int level_idx,
  QGraphicsLineItem* line_item,
  const EditorModeId mode)
{
  clear_selection(level_idx);
#if 0
  if (line_item == nullptr)
    return;

  // find if any of our lanes match those vertices
  for (auto& edge : building.levels[level_idx].edges)
  {
    if (mode == MODE_TRAFFIC)
    {
      if (edge.type != Edge::LANE)
        continue;
      if (edge.get_graph_idx() != traffic_map_idx)
        continue;
    }

    if (mode == MODE_CROWD_SIM)
    {
      if (edge.type != Edge::HUMAN_LANE)
        continue;
      if (edge.get_graph_idx() != traffic_map_idx)
        continue;
    }

    if (mode == MODE_BUILDING &&
      (edge.type == Edge::LANE || edge.type == Edge::HUMAN_LANE) )
      continue;

    // look up the line's vertices
    const double x1 = line_item->line().x1();
    const double y1 = line_item->line().y1();
    const double x2 = line_item->line().x2();
    const double y2 = line_item->line().y2();

    const auto& v_start = building.levels[level_idx].vertices[edge.start_idx];
    const auto& v_end = building.levels[level_idx].vertices[edge.end_idx];

    // calculate distances
    const double dx1 = v_start.x - x1;
    const double dy1 = v_start.y - y1;
    const double dx2 = v_end.x - x2;
    const double dy2 = v_end.y - y2;
    const double v1_dist = std::sqrt(dx1*dx1 + dy1*dy1);
    const double v2_dist = std::sqrt(dx2*dx2 + dy2*dy2);

    const double thresh = 10.0;  // it should be really tiny if it matches
    if (v1_dist < thresh && v2_dist < thresh)
    {
      edge.selected = true;
      return;  // stop after first one is found, don't select multiple
    }
  }
#endif
}

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

Polygon* Project::get_selected_polygon(
  const EditorModeId mode,
  const int level_idx)
{
  if (mode == MODE_BUILDING)
  {
    for (size_t i = 0; i < building.levels[level_idx].polygons.size(); i++)
    {
      if (building.levels[level_idx].polygons[i].selected)
        return &building.levels[level_idx].polygons[i];// abomination
    }
  }
  return nullptr;
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
  const int level_idx,
  const int start_idx,
  const int end_idx)
{
  building.add_lane(level_idx, start_idx, end_idx, traffic_map_idx);
}
