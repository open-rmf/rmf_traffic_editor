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

#ifndef LEVEL_H
#define LEVEL_H

#include <yaml-cpp/yaml.h>
#include <string>

#include "edge.h"
#include "fiducial.h"
#include "layer.h"
#include "model.h"
#include "polygon.h"
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

  // temporary, just for debugging polygon edge projection...
  double polygon_edge_proj_x = 0.0;
  double polygon_edge_proj_y = 0.0;

  virtual bool from_yaml(const std::string& name, const YAML::Node& data) = 0;
  virtual YAML::Node to_yaml() const = 0;

  virtual bool delete_selected() = 0;

  Polygon::EdgeDragPolygon polygon_edge_drag_press(
    const Polygon* polygon,
    const double x,
    const double y);

  virtual void clear_selection() = 0;

  void add_vertex(const double x, const double y);
  size_t get_vertex_by_id(QUuid vertex_id);

protected:
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
};

#endif
