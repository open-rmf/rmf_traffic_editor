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

#include "add_edge.h"

AddEdgeCommand::AddEdgeCommand(
  Building* building,
  int level_idx,
  const RenderingOptions& rendering_options)
{
  _building = building;
  _rendering_options = rendering_options;
  _level_idx = level_idx;
  _first_point_not_exist = false;
  _first_point_drawn = false;
  _second_point_not_exist = false;
  _second_point_drawn = false;
  _edge_snapshot = _building->levels[_level_idx].edges;
  _vert_snapshot = _building->levels[_level_idx].vertices;
}

AddEdgeCommand::~AddEdgeCommand()
{

}

void AddEdgeCommand::redo()
{
  _building->levels[_level_idx].vertices = _final_snapshot;
  if (_type != Edge::LANE)
  {
    _building->add_edge(
      _level_idx,
      _vert_id_first,
      _vert_id_second,
      _type);
  }
  else
  {
    _building->add_lane(
      _level_idx,
      _vert_id_first,
      _vert_id_second,
      _rendering_options.active_traffic_map_idx);
  }
}

void AddEdgeCommand::undo()
{
  //Just use snapshots to keep things simpler
  _building->levels[_level_idx].edges = _edge_snapshot;
  _building->levels[_level_idx].vertices = _vert_snapshot;
}

int AddEdgeCommand::set_first_point(double x, double y)
{
  _first_x = x;
  _first_y = y;

  const double vertex_dist_thresh_pixels =
    _vertex_radius_meters /
    _building->levels[_level_idx].drawing_meters_per_pixel;

  int clicked_idx = _building->nearest_item_index_if_within_distance(
    _level_idx,
    x,
    y,
    vertex_dist_thresh_pixels,
    Level::VERTEX);

  _first_point_drawn = true;
  if (clicked_idx < 0)
  {
    _first_point_not_exist = true;
    _building->add_vertex(_level_idx, x, y);
    clicked_idx = _building->levels[_level_idx].vertices.size()-1;
  }
  _vert_id_first = clicked_idx;
  _final_snapshot = _building->levels[_level_idx].vertices;
  return clicked_idx;
}

int AddEdgeCommand::set_second_point(double x, double y)
{
  _second_x = x;
  _second_y = y;

  const double vertex_dist_thresh_pixels =
    _vertex_radius_meters /
    _building->levels[_level_idx].drawing_meters_per_pixel;

  int clicked_idx = _building->nearest_item_index_if_within_distance(
    _level_idx,
    x,
    y,
    vertex_dist_thresh_pixels,
    Level::VERTEX);

  _second_point_drawn = true;

  if (clicked_idx < 0)
  {
    _second_point_not_exist = true;
    _second_point_drawn = true;
    _building->add_vertex(_level_idx, x, y);
    clicked_idx = _building->levels[_level_idx].vertices.size()-1;
  }
  _vert_id_second = clicked_idx;
  _final_snapshot = _building->levels[_level_idx].vertices;
  return clicked_idx;
}

void AddEdgeCommand::set_edge_type(Edge::Type type)
{
  _type = type;
}
