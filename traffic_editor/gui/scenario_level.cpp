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

#include <QGraphicsOpacityEffect>
#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QImage>
#include <QImageReader>

#include "scenario_level.h"
using std::string;
using std::vector;


ScenarioLevel::ScenarioLevel()
: Level()
{
}

ScenarioLevel::~ScenarioLevel()
{
}

bool ScenarioLevel::from_yaml(
  const std::string& _name,
  const YAML::Node& yaml_node)
{
  printf("parsing scenario level [%s]\n", _name.c_str());

  name = _name;

  if (!yaml_node.IsMap())
    throw std::runtime_error("level " + name + " YAML invalid");

  parse_vertices(yaml_node);

  if (yaml_node["roi"] && yaml_node["roi"].IsSequence())
  {
    const YAML::Node& y_roi = yaml_node["roi"];
    for (YAML::const_iterator it = y_roi.begin(); it != y_roi.end(); ++it)
    {
      Polygon p;
      p.from_yaml(*it, Polygon::ROI);
      polygons.push_back(p);
    }
  }
  return true;
}

YAML::Node ScenarioLevel::to_yaml() const
{
  YAML::Node y;
  for (const auto& v : vertices)
    y["vertices"].push_back(v.to_yaml());

  for (const auto& polygon : polygons)
  {
    switch (polygon.type)
    {
      case Polygon::ROI:
        y["roi"].push_back(polygon.to_yaml());
        break;
      default:
        printf("tried to save an unknown polygon type: %d\n",
          static_cast<int>(polygon.type));
        break;
    }
  }

  return y;
}

bool ScenarioLevel::delete_selected()
{
  // Vertices take a lot more care, because we have to check if a vertex
  // is used in an edge or a polygon before deleting it, and update all
  // higher-index vertex indices in the edges and polygon vertex lists.
  // Since this is a potentially expensive operation, first we'll spin
  // through the vertex list and see if any vertices are selected, and
  // only then make a copy of the vertex list.
  int selected_vertex_idx = -1;
  for (int i = 0; i < static_cast<int>(vertices.size()); i++)
  {
    if (vertices[i].selected)
    {
      selected_vertex_idx = i;
      break;  // just grab the index of the first selected vertex
    }
  }
  if (selected_vertex_idx >= 0)
  {
    // See if this vertex is used in any edges/polygons.
    bool vertex_used = false;
    for (const auto& polygon : polygons)
    {
      for (const int& vertex_idx : polygon.vertices)
      {
        if (vertex_idx == selected_vertex_idx)
          vertex_used = true;
      }
    }
    if (vertex_used)
      return false;// don't try to delete a vertex used in a shape

    // the vertex is not currently being used, so let's erase it
    vertices.erase(vertices.begin() + selected_vertex_idx);

    for (Polygon& polygon : polygons)
    {
      for (int i = 0; i < static_cast<int>(polygon.vertices.size()); i++)
      {
        if (polygon.vertices[i] > selected_vertex_idx)
          polygon.vertices[i]--;
      }
    }
  }
  return true;
}

void ScenarioLevel::draw_polygons(QGraphicsScene* scene) const
{
  QBrush polygon_brush(QColor::fromRgbF(0.8, 0.8, 0.0, 0.2));
  QBrush selected_polygon_brush(QColor::fromRgbF(1.0, 0.0, 0.0, 0.5));

  for (const auto& polygon : polygons)
  {
    // now draw the polygons
    QVector<QPointF> polygon_vertices;
    for (const auto& vertex_idx: polygon.vertices)
    {
      const Vertex& v = vertices[vertex_idx];
      polygon_vertices.append(QPointF(v.x, v.y));
    }
    scene->addPolygon(
      QPolygonF(polygon_vertices),
      QPen(Qt::black),
      polygon.selected ? selected_polygon_brush : polygon_brush);
  }
}

void ScenarioLevel::clear_selection()
{
  for (auto& vertex : vertices)
    vertex.selected = false;

  for (auto& polygon : polygons)
    polygon.selected = false;
}

void ScenarioLevel::draw(
  QGraphicsScene* scene,
  const double meters_per_pixel) const
{
  draw_polygons(scene);

  for (const auto& v : vertices)
    v.draw(scene, 0.1 / meters_per_pixel, QColor::fromRgbF(1.0, 1.0, 0.0));
}
