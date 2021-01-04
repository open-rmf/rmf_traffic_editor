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

#ifndef BUILDING_H
#define BUILDING_H


class QGraphicsScene;

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include <QGraphicsLineItem>
#include <QPointF>

#include "building_level.h"
#include "lift.h"
#include <traffic_editor/crowd_sim/crowd_sim_impl.h>

class Building
{
public:
  Building();
  virtual ~Building();

  std::string name;
  std::string reference_level_name;
  std::vector<BuildingLevel> levels;
  std::vector<Lift> lifts;
  std::mutex building_mutex;

  mutable crowd_sim::CrowdSimImplPtr crowd_sim_impl;

  std::string filename;

  bool load_yaml_file();
  bool save_yaml_file();
  void clear();  // clear all internal data structures

  void add_level(const BuildingLevel& level);

  void add_vertex(int level_index, double x, double y);
  QUuid add_fiducial(int level_index, double x, double y);

  int find_nearest_vertex_index(
    int level_index, double x, double y, double& distance);

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

  void add_lane(
    const int level_idx,
    const int start_idx,
    const int end_idx,
    const int graph_idx);

  QUuid add_model(
    const int level_idx,
    const double x,
    const double y,
    const double z,
    const double yaw,
    const std::string& model_name);

  bool delete_selected(const int level_index);

  void set_model_yaw(
    const int level_idx,
    const int model_idx,
    const double yaw);

  void draw_lifts(QGraphicsScene* scene, const int level_idx);

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

  int get_reference_level_idx();

  void clear_scene();

  double level_meters_per_pixel(const std::string& level_name) const;

  void rotate_all_models(const double rotation);
};

#endif
