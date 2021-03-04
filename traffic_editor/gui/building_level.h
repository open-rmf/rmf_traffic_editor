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

#ifndef BUILDING_LEVEL_H
#define BUILDING_LEVEL_H

#include "level.h"

#include <yaml-cpp/yaml.h>
#include <string>

#include "correspondence_point.hpp"
#include "edge.h"
#include "editor_model.h"
#include "fiducial.h"
#include "layer.h"
#include "model.h"
#include "polygon.h"
#include "rendering_options.h"
#include "vertex.h"
#include <traffic_editor/crowd_sim/crowd_sim_impl.h>

#include <QPixmap>
#include <QPainterPath>
class QGraphicsScene;


class BuildingLevel : public Level
{
public:
  BuildingLevel();
  ~BuildingLevel();

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

  QPixmap floorplan_pixmap;

  mutable crowd_sim::CrowdSimImplPtr crowd_sim_impl;

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

  void set_drawing_visible(bool value) { drawing_visible_ = value; }

  const std::vector<std::vector<CorrespondencePoint>>&
  correspondence_point_sets() const
  {
    return correspondence_point_sets_;
  }

  std::vector<std::vector<CorrespondencePoint>>& correspondence_point_sets()
  {
    return correspondence_point_sets_;
  }

  void layer_added();
  void set_active_layer(int active_layer) { active_layer_ = active_layer; }
  QUuid add_correspondence_point(int layer, double x, double y);
  bool export_correspondence_points(const std::string& filename) const;

private:
  bool drawing_visible_ = true;
  std::vector<std::vector<CorrespondencePoint>> correspondence_point_sets_;
  std::vector<int> next_cp_ids;
  int active_layer_ = 0;

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
