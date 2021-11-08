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
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include <QGraphicsLineItem>
#include <QPointF>

#include "coordinate_system.h"
#include "graph.h"
#include "level.h"
#include "lift.h"
#include "param.h"
#include <traffic_editor/crowd_sim/crowd_sim_impl.h>
#include "rendering_options.h"

class Building
{
public:
  Building();
  virtual ~Building();

  std::string name;
  std::string reference_level_name;
  std::vector<Level> levels;
  std::vector<Lift> lifts;
  std::vector<Graph> graphs;
  std::map<std::string, Param> params;
  CoordinateSystem coordinate_system;

  mutable crowd_sim::CrowdSimImplPtr crowd_sim_impl;

  bool set_filename(const std::string& _filename);
  std::string get_filename() { return filename; }

  bool load(const std::string& filename);
  bool save();
  void clear();  // clear all internal data structures

  bool export_features(
    int level_index,
    const std::string& dest_filename) const;

  void clear_selection(const int level_idx);
  bool can_delete_current_selection(const int level_idx);

  void add_level(const Level& level);

  void add_vertex(int level_index, double x, double y);
  QUuid add_fiducial(int level_index, double x, double y);
  QUuid add_feature(int level, int layer, double x, double y);
  void remove_feature(const int level, const int layer, QUuid feature_uuid);

  void add_constraint(const int level_idx, const QUuid& a, const QUuid& b);
  void remove_constraint(const int level_idx, const QUuid& a, const QUuid& b);

  int nearest_item_index_if_within_distance(
    const double level_idx,
    const double x,
    const double y,
    const double distance_threshold,
    const Level::ItemType item_type);

  int find_nearest_vertex_index(
    int level_index, double x, double y, double& distance);

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

  void get_selected_items(const int level_idx,
    std::vector<Level::SelectedItem>& selected);

  void draw(
    QGraphicsScene* scene,
    const int level_idx,
    std::vector<EditorModel>& editor_models,
    const RenderingOptions& rendering_options);

  /*
  void mouse_select_press(
    const int level_idx,
    const double x,
    const double y,
    QGraphicsItem* graphics_item,
    const RenderingOptions& rendering_options);
  */

  Polygon* get_selected_polygon(const int level_idx);

  Polygon::EdgeDragPolygon polygon_edge_drag_press(
    const int level_idx,
    const Polygon* polygon,
    const double x,
    const double y);

private:
  std::string filename;
};

#endif
