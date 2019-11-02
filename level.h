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

#ifndef LEVEL_H
#define LEVEL_H

#include <yaml-cpp/yaml.h>
#include <string>

#include "vertex.h"
#include "edge.h"
#include "model.h"
#include "polygon.h"

#include <QPixmap>
class QGraphicsScene;


class Level
{
public:
  Level();
  ~Level();

  std::string name;
  std::string drawing_filename;
  int drawing_width, drawing_height;
  double drawing_meters_per_pixel;
  double elevation;

  double x_meters, y_meters;  // manually specified if no drawing supplied

  std::vector<Vertex> vertices;
  std::vector<Edge> edges;
  std::vector<Model> models;
  std::vector<Polygon> polygons;
  QPixmap pixmap;

  // temporary, just for debugging polygon edge projection...
  double polygon_edge_proj_x, polygon_edge_proj_y;

  bool from_yaml(const std::string &name, const YAML::Node &data);
  YAML::Node to_yaml() const;

  void delete_keypress();
  void calculate_scale();

  void remove_polygon_vertex(const int polygon_idx, const int vertex_idx);

  int polygon_edge_drag_press(
      const int polygon_idx,
      const double x,
      const double y);

  void draw_edges(QGraphicsScene *scene) const;
  void draw_vertices(QGraphicsScene *scene) const;

private:
  double point_to_line_segment_distance(
      const double x,
      const double y,
      const double x0,
      const double y0,
      const double x1,
      const double y1,
      double &x_proj,
      double &y_proj);

  void draw_lane(QGraphicsScene *scene, const Edge &edge) const;
  void draw_wall(QGraphicsScene *scene, const Edge &edge) const;
  void draw_meas(QGraphicsScene *scene, const Edge &edge) const;
};

#endif
