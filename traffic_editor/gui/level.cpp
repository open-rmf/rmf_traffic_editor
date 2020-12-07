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

#include <algorithm>
#include <cmath>

#include <QGraphicsScene>
#include <QImage>
#include <QImageReader>

#include "traffic_editor/level.h"
using std::string;
using std::vector;


Level::Level()
{
}

Level::~Level()
{
}

void Level::load_yaml_edge_sequence(
  const YAML::Node& data,
  const char* sequence_name,
  const Edge::Type type)
{
  if (!data[sequence_name] || !data[sequence_name].IsSequence())
    return;

  const YAML::Node& yl = data[sequence_name];
  for (YAML::const_iterator it = yl.begin(); it != yl.end(); ++it)
  {
    Edge e;
    e.from_yaml(*it, type);
    edges.push_back(e);
  }
}

double Level::point_to_line_segment_distance(
  const double x, const double y,
  const double x0, const double y0,
  const double x1, const double y1,
  double& x_proj, double& y_proj)
{
  // this portion figures out which edge is closest to (x, y) by repeatedly
  // testing the distance from the click to each edge in the polygon, using
  // geometry similar to that explained in:
  // https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment

  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double segment_length_squared = dx*dx + dy*dy;

  const double dx0 = x - x0;
  const double dy0 = y - y0;
  const double dot = dx0*dx + dy0*dy;
  const double t = std::max(
    0.0,
    std::min(1.0, dot / segment_length_squared));

  x_proj = x0 + t * dx;
  y_proj = y0 + t * dy;

  const double dx_proj = x - x_proj;
  const double dy_proj = y - y_proj;

  const double dist = std::sqrt(dx_proj * dx_proj + dy_proj * dy_proj);

  /*
  printf("   p=(%.1f, %.1f) p0=(%.1f, %.1f) p1=(%.1f, %.1f) t=%.3f proj=(%.1f, %.1f) dist=%.3f\n",
      x, y, x0, y0, x1, y1, t, x_proj, y_proj, dist);
  */

  return dist;
}

/*
 * This function returns the index of the polygon vertex that will be
 * 'split' by the newly created edge
 */
Polygon::EdgeDragPolygon Level::polygon_edge_drag_press(
  const Polygon* polygon,
  const double x,
  const double y)
{
  Polygon::EdgeDragPolygon edp;

  if (polygon == nullptr || polygon->vertices.empty())
    return edp;

  // cruise along all possible line segments and calculate the distance
  // to this point

  int min_idx = 0;
  double min_dist = 1.0e9;

  for (size_t v0_idx = 0; v0_idx < polygon->vertices.size(); v0_idx++)
  {
    const size_t v1_idx =
      v0_idx < polygon->vertices.size() - 1 ? v0_idx + 1 : 0;
    const size_t v0 = polygon->vertices[v0_idx];
    const size_t v1 = polygon->vertices[v1_idx];

    const double x0 = vertices[v0].x;
    const double y0 = vertices[v0].y;
    const double x1 = vertices[v1].x;
    const double y1 = vertices[v1].y;

    double x_proj = 0, y_proj = 0;
    const double dist = point_to_line_segment_distance(
      x, y, x0, y0, x1, y1, x_proj, y_proj);

    if (dist < min_dist)
    {
      min_idx = v0;
      min_dist = dist;

      // save the nearest projected point to help debug this visually
      polygon_edge_proj_x = x_proj;
      polygon_edge_proj_y = y_proj;
    }
  }

  // create the mouse motion polygon and insert a new edge
  QVector<QPointF> polygon_vertices;
  for (size_t i = 0; i < polygon->vertices.size(); i++)
  {
    const int v_idx = polygon->vertices[i];
    const Vertex& v = vertices[v_idx];
    polygon_vertices.append(QPointF(v.x, v.y));
    if (v_idx == min_idx)
    {
      polygon_vertices.append(QPointF(x, y));  // current mouse location
      edp.movable_vertex = i + 1;
    }
  }
  edp.polygon = QPolygonF(polygon_vertices);

  return edp;
}

bool Level::parse_vertices(const YAML::Node& _data)
{
  if (_data["vertices"] && _data["vertices"].IsSequence())
  {
    const YAML::Node& pts = _data["vertices"];
    for (YAML::const_iterator it = pts.begin(); it != pts.end(); ++it)
    {
      Vertex v;
      v.from_yaml(*it);
      vertices.push_back(v);
    }
  }
  return true;
}

void Level::add_vertex(const double x, const double y)
{
  vertices.push_back(Vertex(x, y));
}

size_t Level::get_vertex_by_id(QUuid vertex_id)
{
  for (size_t i = 0; i < vertices.size(); i++)
  {
    if (vertices[i].uuid == vertex_id)
    {
      return i;
    }
  }
  return vertices.size()+1;
}
