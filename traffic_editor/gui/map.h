/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef NAV_MAP_H
#define NAV_MAP_H

#include <string>
#include <vector>

#include "level.h"


class Map
{
public:
  Map();
  ~Map();

  void load_yaml(const std::string &filename);
  bool save_yaml(const std::string &filename);
  void clear();  // clear all internal data structures

  std::string building_name;
  std::vector<Level> levels;
  bool changed;  // true if map changed since last save/open

  void add_level(const Level &level);

  void add_vertex(int level_index, double x, double y);
  int find_nearest_vertex_index(
      int level_index, double x, double y, double &distance);

  enum ItemType { VERTEX=1, MODEL };

  int nearest_item_index_if_within_distance(
      const int level_index,
      const double x,
      const double y,
      const double distance_threshold,
      const ItemType item_type);

  void add_edge(
      const int level_idx,
      const int start_idx,
      const int end_idx,
      const Edge::Type edge_type);

  void add_model(
      const int level_idx,
      const double x,
      const double y,
      const double yaw,
      const std::string &model_name);

  void delete_keypress(const int level_index);

  void rotate_model(
      const int level_idx,
      const int model_idx,
      const double release_x,
      const double release_y);

  void remove_polygon_vertex(
      const int level_idx,
      const int polygon_idx,
      const int vertex_idx);

  int polygon_edge_drag_press(
      const int level_idx,
      const int polygon_idx,
      const double x,
      const double y);
};

#endif
