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


class QGraphicsScene;

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include <QPointF>

#include "level.h"
#include "lift.h"


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
  std::vector<Lift> lifts;
  bool changed;  // true if map changed since last save/open

  void add_level(const Level &level);

  void add_vertex(int level_index, double x, double y);
  void add_fiducial(int level_index, double x, double y);

  int find_nearest_vertex_index(
      int level_index, double x, double y, double &distance);

  enum ItemType { VERTEX=1, MODEL, FIDUCIAL };
  struct NearestItem
  {
    double model_dist = 1e100;
    int model_idx = -1;

    double vertex_dist = 1e100;
    int vertex_idx = -1;

    double fiducial_dist = 1e100;
    int fiducial_idx = -1;
  };

  NearestItem nearest_items(
      const int level_index,
      const double x,
      const double y);

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

  bool delete_selected(const int level_index);

  void set_model_yaw(
      const int level_idx,
      const int model_idx,
      const double yaw);

  void remove_polygon_vertex(
      const int level_idx,
      const int polygon_idx,
      const int vertex_idx);

  int polygon_edge_drag_press(
      const int level_idx,
      const int polygon_idx,
      const double x,
      const double y);

  void draw_lifts(QGraphicsScene *scene, const int level_idx);

  bool transform_between_levels(
      const std::string& from_level_name,
      const QPointF& from_point,
      const std::string& to_level_name,
      QPointF& to_point);

  bool transform_between_levels(
      const int from_level_idx,
      const QPointF& from_point,
      const int to_level_idx,
      QPointF& to_point);

  void clear_transform_cache();

  struct LevelPair
  {
    int from_idx = -1;
    int to_idx = -1;

    bool operator<(const LevelPair& rhs) const
    {
      return std::tie(from_idx, to_idx) < std::tie(rhs.from_idx, rhs.to_idx);
    }
  };

  // to apply transform: first scale, then translate
  struct Transform
  {
    double scale = 1.0;
    double dx = 0.0;
    double dy = 0.0;
  };
  typedef std::map<LevelPair, Transform> TransformMap;
  TransformMap transforms;

  Transform compute_transform(
      const int from_level_idx,
      const int to_level_idx);

  Transform get_transform(
      const int from_level_idx,
      const int to_level_idx);

  void calculate_all_transforms();

private:
  // Recursive function to write YAML ordered maps. Credit: Dave Hershberger
  void write_yaml_node(const YAML::Node& node, YAML::Emitter& emitter);
};

#endif
