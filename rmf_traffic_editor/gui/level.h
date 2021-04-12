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

#ifndef LEVEL_H
#define LEVEL_H

#include <yaml-cpp/yaml.h>
#include <string>

#include "edge.h"
#include "editor_model.h"
#include "feature.hpp"
#include "fiducial.h"
#include "layer.h"
#include "model.h"
#include "polygon.h"
#include "rendering_options.h"
#include "vertex.h"

#include <QPixmap>
#include <QPainterPath>
class QGraphicsScene;


class Level
{
public:
  Level();
  ~Level();

  std::string name;

  std::vector<Vertex> vertices;
  std::vector<Edge> edges;
  std::vector<Polygon> polygons;

  std::vector<Layer> layers;
  bool are_layer_names_unique();

  // temporary, just for debugging polygon edge projection...
  double polygon_edge_proj_x = 0.0;
  double polygon_edge_proj_y = 0.0;

  Polygon::EdgeDragPolygon polygon_edge_drag_press(
    const Polygon* polygon,
    const double x,
    const double y);

  void add_vertex(const double x, const double y);
  size_t get_vertex_by_id(QUuid vertex_id);

  std::string drawing_filename;
  int drawing_width = 0;
  int drawing_height = 0;
  double drawing_meters_per_pixel = 0.05;
  double elevation = 0.0;
  const double vertex_radius = 0.1;  // meters

  double x_meters = 10.0;  // manually specified if no drawing supplied
  double y_meters = 10.0;  // manually specified if no drawing supplied

  // when generating the building in "flattened" mode, the levels have
  // to be offset in the (x, y) plane to avoid clobbering each other.
  double flattened_x_offset = 0.0;
  double flattened_y_offset = 0.0;

  std::vector<Model> models;
  std::vector<Fiducial> fiducials;
  std::vector<Feature> floorplan_features;

  QPixmap floorplan_pixmap;

  bool from_yaml(const std::string& name, const YAML::Node& data);
  YAML::Node to_yaml() const;

  struct SelectedItem
  {
    int model_idx = -1;
    int vertex_idx = -1;
    int fiducial_idx = -1;
    int edge_idx = -1;
    int polygon_idx = -1;
  };

  bool can_delete_current_selection();
  bool delete_selected();
  void calculate_scale();
  void clear_selection();
  void get_selected_items(std::vector<SelectedItem>& selected_items);


  void draw(
    QGraphicsScene* scene,
    std::vector<EditorModel>& editor_models,
    const RenderingOptions& rendering_options);

  void clear_scene();

  bool load_drawing();

  void set_drawing_visible(bool value) { _drawing_visible = value; }
  bool get_drawing_visible() const { return _drawing_visible; }

  const std::vector<std::vector<Feature>>& feature_sets() const
  {
    return _feature_sets;
  }

  std::vector<std::vector<Feature>>& feature_sets()
  {
    return _feature_sets;
  }

  void layer_added();
  void set_active_layer(int active_layer) { _active_layer = active_layer; }
  QUuid add_feature(int layer, double x, double y);
  bool export_features(const std::string& filename) const;

private:
  double point_to_line_segment_distance(
    const double x,
    const double y,
    const double x0,
    const double y0,
    const double x1,
    const double y1,
    double& x_proj,
    double& y_proj);

  void load_yaml_edge_sequence(
    const YAML::Node& data,
    const char* sequence_name,
    const Edge::Type type);

  bool parse_vertices(const YAML::Node& _data);

  bool _drawing_visible = true;
  std::vector<std::vector<Feature>> _feature_sets;
  std::vector<int> _next_cp_ids;
  int _active_layer = 0;

  void draw_lane(
    QGraphicsScene* scene,
    const Edge& edge,
    const RenderingOptions& rendering_options) const;

  void draw_wall(QGraphicsScene* scene, const Edge& edge) const;
  void draw_meas(QGraphicsScene* scene, const Edge& edge) const;
  void draw_door(QGraphicsScene* scene, const Edge& edge) const;
  void draw_fiducials(QGraphicsScene* scene) const;
  void draw_polygons(QGraphicsScene* scene) const;

  // helper function
  void draw_polygon(
    QGraphicsScene* scene,
    const QBrush& brush,
    const Polygon& polygon) const;

  void add_door_swing_path(
    QPainterPath& path,
    double hinge_x,
    double hinge_y,
    double door_length,
    double start_angle,
    double end_angle) const;

  void add_door_slide_path(
    QPainterPath& path,
    double hinge_x,
    double hinge_y,
    double door_length,
    double door_angle) const;

};

#endif
