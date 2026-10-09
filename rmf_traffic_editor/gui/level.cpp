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

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>

#include "ceres/ceres.h"
#include <QGraphicsOpacityEffect>
#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QImage>
#include <QImageReader>

#include "level.h"
#include "yaml_utils.h"

using std::string;
using std::vector;


Level::Level()
{
}

Level::~Level()
{
}

bool Level::from_yaml(
  const std::string& _name,
  const YAML::Node& _data,
  const CoordinateSystem& coordinate_system)
{
  printf("parsing level [%s]\n", _name.c_str());
  name = _name;

  if (!_data.IsMap())
    throw std::runtime_error("level " + name + " YAML invalid");

  if (_data["drawing"] && _data["drawing"].IsMap())
  {
    const YAML::Node& drawing_data = _data["drawing"];
    if (!drawing_data["filename"])
      throw std::runtime_error("level " + name + " drawing invalid");
    drawing_filename = drawing_data["filename"].as<string>();
    /*
    if (!load_drawing())
      return false;
    */
  }
  else if (_data["x_meters"] && _data["y_meters"])
  {
    x_meters = _data["x_meters"].as<double>();
    y_meters = _data["y_meters"].as<double>();
    drawing_meters_per_pixel = coordinate_system.default_scale();
    drawing_width = x_meters / drawing_meters_per_pixel;
    drawing_height = y_meters / drawing_meters_per_pixel;
  }
  else
  {
    x_meters = 100.0;
    y_meters = 100.0;
    drawing_meters_per_pixel = coordinate_system.default_scale();
    drawing_width = x_meters / drawing_meters_per_pixel;
    drawing_height = y_meters / drawing_meters_per_pixel;
  }

  if (_data["vertices"] && _data["vertices"].IsSequence())
  {
    const YAML::Node& pts = _data["vertices"];
    for (YAML::const_iterator it = pts.begin(); it != pts.end(); ++it)
    {
      Vertex v;
      v.from_yaml(*it, coordinate_system);
      vertices.push_back(v);
    }
  }

  if (_data["fiducials"] && _data["fiducials"].IsSequence())
  {
    const YAML::Node& fy = _data["fiducials"];
    for (YAML::const_iterator it = fy.begin(); it != fy.end(); ++it)
    {
      Fiducial f;
      f.from_yaml(*it);
      fiducials.push_back(f);
    }
  }

  if (_data["features"] && _data["features"].IsSequence())
  {
    const YAML::Node& fy = _data["features"];
    for (YAML::const_iterator it = fy.begin(); it != fy.end(); ++it)
    {
      Feature f;
      f.from_yaml(*it);
      floorplan_features.push_back(f);
    }
  }

  if (_data["constraints"] && _data["constraints"].IsSequence())
  {
    const YAML::Node& c_data = _data["constraints"];
    for (YAML::const_iterator it = c_data.begin(); it != c_data.end(); ++it)
    {
      Constraint c;
      c.from_yaml(*it);
      constraints.push_back(c);
    }
  }

  load_yaml_edge_sequence(_data, "lanes", Edge::LANE);
  load_yaml_edge_sequence(_data, "walls", Edge::WALL);
  load_yaml_edge_sequence(_data, "measurements", Edge::MEAS);
  load_yaml_edge_sequence(_data, "doors", Edge::DOOR);
  load_yaml_edge_sequence(_data, "human_lanes", Edge::HUMAN_LANE);

  if (_data["models"] && _data["models"].IsSequence())
  {
    const YAML::Node& ys = _data["models"];
    for (YAML::const_iterator it = ys.begin(); it != ys.end(); ++it)
    {
      Model m;
      m.from_yaml(*it, this->name, coordinate_system);
      models.push_back(m);
    }
  }

  if (_data["floors"] && _data["floors"].IsSequence())
  {
    const YAML::Node& yf = _data["floors"];
    for (YAML::const_iterator it = yf.begin(); it != yf.end(); ++it)
    {
      Polygon p;
      p.from_yaml(*it, Polygon::FLOOR);
      polygons.push_back(p);
    }
  }

  if (_data["holes"] && _data["holes"].IsSequence())
  {
    const YAML::Node& yf = _data["holes"];
    for (YAML::const_iterator it = yf.begin(); it != yf.end(); ++it)
    {
      Polygon p;
      p.from_yaml(*it, Polygon::HOLE);
      polygons.push_back(p);
    }
  }

  if (_data["rois"] && _data["rois"].IsSequence())
  {
    const YAML::Node& yf = _data["rois"];
    for (YAML::const_iterator it = yf.begin(); it != yf.end(); ++it)
    {
      Polygon p;
      p.from_yaml(*it, Polygon::ROI);
      polygons.push_back(p);
    }
  }

  if (_data["elevation"])
    elevation = _data["elevation"].as<double>();

  if (_data["layers"] && _data["layers"].IsMap())
  {
    const YAML::Node& yl = _data["layers"];
    for (YAML::const_iterator it = yl.begin(); it != yl.end(); ++it)
    {
      Layer layer;
      layer.from_yaml(it->first.as<string>(), it->second, coordinate_system);
      layers.push_back(layer);
    }
  }

  return true;
}

bool Level::load_drawing()
{
  if (drawing_filename.empty())
    return true;// nothing to load

  printf("  level %s drawing: %s\n",
    name.c_str(),
    drawing_filename.c_str());

  QString qfilename = QString::fromStdString(drawing_filename);

  QImageReader image_reader(qfilename);
  image_reader.setAutoTransform(true);
  QImage image = image_reader.read();
  if (image.isNull())
  {
    qWarning("unable to read %s: %s",
      qUtf8Printable(qfilename),
      qUtf8Printable(image_reader.errorString()));
    return false;
  }
  image = image.convertToFormat(QImage::Format_Grayscale8);
  floorplan_pixmap = QPixmap::fromImage(image);
  drawing_width = floorplan_pixmap.width();
  drawing_height = floorplan_pixmap.height();
  return true;
}

YAML::Node Level::to_yaml(const CoordinateSystem& coordinate_system) const
{
  YAML::Node y;
  if (!drawing_filename.empty())
  {
    YAML::Node drawing_node;
    drawing_node["filename"] = drawing_filename;
    y["drawing"] = drawing_node;
  }
  else
  {
    y["x_meters"] = x_meters;
    y["y_meters"] = y_meters;
  }
  y["elevation"] = elevation;

  for (const auto& v : vertices)
    y["vertices"].push_back(v.to_yaml(coordinate_system));

  for (const auto& feature : floorplan_features)
    y["features"].push_back(feature.to_yaml());

  for (const auto& constraint : constraints)
    y["constraints"].push_back(constraint.to_yaml());

  for (const auto& f : fiducials)
    y["fiducials"].push_back(f.to_yaml());

  for (const auto& edge : edges)
  {
    YAML::Node n(edge.to_yaml());
    std::string dict_name = "unknown";
    switch (edge.type)
    {
      case Edge::LANE:
        dict_name = "lanes";
        break;
      case Edge::WALL:
        dict_name = "walls";
        break;
      case Edge::MEAS:
        dict_name = "measurements";
        break;
      case Edge::DOOR:
        dict_name = "doors";
        break;
      case Edge::HUMAN_LANE:
        dict_name = "human_lanes";
        break;
      default:
        printf("tried to save unknown edge type: %d\n",
          static_cast<int>(edge.type));
        break;
    }
    y[dict_name].push_back(n);
  }

  for (const auto& model : models)
    y["models"].push_back(model.to_yaml(coordinate_system));

  for (const auto& polygon : polygons)
  {
    switch (polygon.type)
    {
      case Polygon::FLOOR:
        y["floors"].push_back(polygon.to_yaml());
        break;
      case Polygon::HOLE:
        y["holes"].push_back(polygon.to_yaml());
        break;
      case Polygon::ROI:
        y["rois"].push_back(polygon.to_yaml());
        break;
      default:
        printf("tried to save an unknown polygon type: %d\n",
          static_cast<int>(polygon.type));
        break;
    }
  }

  y["layers"] = YAML::Node(YAML::NodeType::Map);
  for (const auto& layer : layers)
    y["layers"][layer.name] = layer.to_yaml(coordinate_system);

  return y;
}

bool Level::can_delete_current_selection()
{
  // if a feature is selected, refuse to delete it if it's in a constraint
  for (const Feature& feature : floorplan_features)
  {
    if (!feature.selected())
      continue;
    for (const Constraint& constraint : constraints)
    {
      if (constraint.includes_id(feature.id()))
        return false;
    }
  }

  for (const Layer& layer : layers)
  {
    for (const Feature& feature : layer.features)
    {
      if (!feature.selected())
        continue;

      for (const Constraint& constraint : constraints)
      {
        if (constraint.includes_id(feature.id()))
          return false;
      }
    }
  }

  int selected_vertex_idx = -1;
  for (int i = 0; i < static_cast<int>(vertices.size()); i++)
  {
    if (vertices[i].selected)
    {
      selected_vertex_idx = i;
      break;  // just grab the index of the first selected vertex
    }
  }

  if (selected_vertex_idx < 0)
    return true;

  bool vertex_used = false;
  for (const auto& edge : edges)
  {
    if (edge.start_idx == selected_vertex_idx ||
      edge.end_idx == selected_vertex_idx)
      vertex_used = true;
  }
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

  /// check if this is a lift_cabin waypoint
  const auto v = vertices[selected_vertex_idx];
  auto it = v.params.find("lift_cabin");
  if ((it != v.params.end()))
  {
    printf("This waypoint is used by a lift cabin!!");
    return false;
  }

  return true;
}

bool Level::delete_selected()
{
  edges.erase(
    std::remove_if(
      edges.begin(),
      edges.end(),
      [](const Edge& edge) { return edge.selected; }),
    edges.end());

  models.erase(
    std::remove_if(
      models.begin(),
      models.end(),
      [](const auto& model) { return model.selected; }),
    models.end());

  fiducials.erase(
    std::remove_if(
      fiducials.begin(),
      fiducials.end(),
      [](const Fiducial& fiducial) { return fiducial.selected; }),
    fiducials.end());

  polygons.erase(
    std::remove_if(
      polygons.begin(),
      polygons.end(),
      [](const Polygon& polygon) { return polygon.selected; }),
    polygons.end());

  constraints.erase(
    std::remove_if(
      constraints.begin(),
      constraints.end(),
      [](const Constraint& constraint) { return constraint.selected(); }),
    constraints.end());

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
    for (const auto& edge : edges)
    {
      if (edge.start_idx == selected_vertex_idx ||
        edge.end_idx == selected_vertex_idx)
        vertex_used = true;
    }
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

    // now go through all edges and polygons to decrement any larger indices
    for (Edge& edge : edges)
    {
      if (edge.start_idx > selected_vertex_idx)
        edge.start_idx--;
      if (edge.end_idx > selected_vertex_idx)
        edge.end_idx--;
    }

    for (Polygon& polygon : polygons)
    {
      for (int i = 0; i < static_cast<int>(polygon.vertices.size()); i++)
      {
        if (polygon.vertices[i] > selected_vertex_idx)
          polygon.vertices[i]--;
      }
    }
  }

  // if a feature is selected, refuse to delete it if it's in a constraint
  for (std::size_t i = 0; i < floorplan_features.size(); i++)
  {
    if (!floorplan_features[i].selected())
      continue;

    for (std::size_t j = 0; j < constraints.size(); j++)
    {
      if (constraints[j].includes_id(floorplan_features[i].id()))
        return false;
    }

    floorplan_features.erase(floorplan_features.begin() + i);
    return true;
  }

  for (std::size_t layer_idx = 0; layer_idx < layers.size(); layer_idx++)
  {
    Layer& layer = layers[layer_idx];
    for (std::size_t i = 0; i < layer.features.size(); i++)
    {
      if (!layer.features[i].selected())
        continue;

      for (std::size_t j = 0; j < constraints.size(); j++)
      {
        if (constraints[j].includes_id(layer.features[i].id()))
          return false;
      }

      layer.features.erase(layer.features.begin() + i);
      return true;
    }
  }

  return true;
}

bool Level::delete_used_entities(int selected_vertex_idx)
{
  if (selected_vertex_idx >= 0)
  {
    edges.erase(
      std::remove_if(
        edges.begin(),
        edges.end(),
        [selected_vertex_idx](const Edge& edge)
        {
          return edge.contains(selected_vertex_idx);
        }),
      edges.end());

    polygons.erase(
      std::remove_if(
        polygons.begin(),
        polygons.end(),
        [selected_vertex_idx](const Polygon& polygon)
        {
          return polygon.contains_vertex(selected_vertex_idx);
        }),
      polygons.end());

    return true;
  }
  return false;
}

bool Level::delete_lift_vertex(std::string lift_name)
{
  std::vector<int> selected_vertex_idxes;
  for (int i = 0; i < static_cast<int>(vertices.size()); i++)
  {
    if (vertices[i].params.count("lift_cabin"))
    {
      if (vertices[i].params["lift_cabin"].value_string == lift_name)
      {
        selected_vertex_idxes.push_back(i);
      }
    }
  }

  for (const auto& idx : selected_vertex_idxes)
  {
    edges.erase(
      std::remove_if(
        edges.begin(),
        edges.end(),
        [idx](const Edge& edge)
        {
          if (edge.start_idx == idx ||
          edge.end_idx == idx)
          {
            return true;
          }
          else
            return false;
        }),
      edges.end());

    for (Edge& edge : edges)
    {
      if (edge.start_idx > idx)
        edge.start_idx--;
      if (edge.end_idx > idx)
        edge.end_idx--;
    }
    vertices.erase(vertices.begin() + idx);
  }
  return true;
}

std::vector<Edge> Level::edges_with_vertex(int vertex_idx) const
{
  std::vector<Edge> result;
  for (const auto& edge : edges)
  {
    if (edge.contains(vertex_idx))
      result.push_back(edge);
  }
  return result;
}

std::vector<Polygon> Level::polygons_with_vertex(int vertex_idx) const
{
  std::vector<Polygon> result;
  for (const auto& polygon : polygons)
  {
    if (polygon.contains_vertex(vertex_idx))
      result.push_back(polygon);
  }
  return result;
}

void Level::get_selected_items(
  std::vector<Level::SelectedItem>& items)
{
  for (std::size_t i = 0; i < edges.size(); i++)
  {
    if (edges[i].selected)
    {
      Level::SelectedItem item;
      item.edge_idx = i;
      items.push_back(item);
    }
  }

  for (std::size_t i = 0; i < models.size(); i++)
  {
    if (models[i].selected)
    {
      Level::SelectedItem item;
      item.model_idx = i;
      items.push_back(item);
    }
  }

  for (std::size_t i = 0; i < vertices.size(); i++)
  {
    if (vertices[i].selected)
    {
      Level::SelectedItem item;
      item.vertex_idx = i;
      items.push_back(item);
    }
  }

  for (std::size_t i = 0; i < fiducials.size(); i++)
  {
    if (fiducials[i].selected)
    {
      Level::SelectedItem item;
      item.fiducial_idx = i;
      items.push_back(item);
    }
  }

  for (std::size_t i = 0; i < polygons.size(); i++)
  {
    if (polygons[i].selected)
    {
      Level::SelectedItem item;
      item.polygon_idx = i;
      items.push_back(item);
    }
  }

  for (std::size_t i = 0; i < floorplan_features.size(); i++)
  {
    if (floorplan_features[i].selected())
    {
      Level::SelectedItem item;
      item.feature_idx = i;
      item.feature_layer_idx = 0;
      items.push_back(item);
    }
  }

  for (std::size_t layer_idx = 0; layer_idx < layers.size(); layer_idx++)
  {
    for (std::size_t i = 0; i < layers[layer_idx].features.size(); i++)
    {
      if (layers[layer_idx].features[i].selected())
      {
        Level::SelectedItem item;
        item.feature_idx = i;
        item.feature_layer_idx = layer_idx + 1;
        items.push_back(item);
      }
    }
  }

  for (std::size_t i = 0; i < constraints.size(); i++)
  {
    if (constraints[i].selected())
    {
      Level::SelectedItem item;
      item.constraint_idx = i;
      items.push_back(item);
    }
  }
}

void Level::calculate_scale(const CoordinateSystem& coordinate_system)
{
  // for now, just calculate the mean of the scale estimates
  double scale_sum = 0.0;
  int scale_count = 0;

  for (auto& edge : edges)
  {
    if (edge.type == Edge::MEAS)
    {
      scale_count++;
      const double dx = vertices[edge.start_idx].x - vertices[edge.end_idx].x;
      const double dy = vertices[edge.start_idx].y - vertices[edge.end_idx].y;
      const double distance_pixels = std::sqrt(dx*dx + dy*dy);
      // todo: a clean, strongly-typed parameter API for edges
      const double distance_meters =
        edge.params[std::string("distance")].value_double;
      scale_sum += distance_meters / distance_pixels;
    }
  }

  if (scale_count > 0)
  {
    drawing_meters_per_pixel = scale_sum / static_cast<double>(scale_count);
    printf("used %d measurements to estimate meters/pixel as %.5f\n",
      scale_count, drawing_meters_per_pixel);
  }
  else
    drawing_meters_per_pixel = coordinate_system.default_scale();

  if (drawing_width && drawing_height && drawing_meters_per_pixel > 0.0)
  {
    x_meters = drawing_width * drawing_meters_per_pixel;
    y_meters = drawing_height * drawing_meters_per_pixel;
  }
}

// todo: migrate this to the TrafficMap class eventually
void Level::draw_lane(
  QGraphicsScene* scene,
  const Edge& edge,
  const RenderingOptions& opts,
  const vector<Graph>& graphs) const
{
  const int graph_idx = edge.get_graph_idx();
  if (graph_idx >= 0 &&
    graph_idx < static_cast<int>(opts.show_building_lanes.size()) &&
    !opts.show_building_lanes[graph_idx])
    return;// don't render this lane

  const auto& v_start = vertices[edge.start_idx];
  const auto& v_end = vertices[edge.end_idx];
  const double dx = v_end.x - v_start.x;
  const double dy = v_end.y - v_start.y;
  const double len = std::sqrt(dx*dx + dy*dy);

  // see if there is a default width for this graph_idx
  double graph_default_width = -1.0;
  for (const auto& graph : graphs)
  {
    if (graph.idx == graph_idx)
    {
      graph_default_width = graph.default_lane_width;
      break;
    }
  }

  double lane_width_meters = 1.0;
  if (edge.get_width() > 0)
    lane_width_meters = edge.get_width();
  else if (graph_default_width > 0)
    lane_width_meters = graph_default_width;

  const double lane_pen_width = lane_width_meters / drawing_meters_per_pixel;
  const double norm_x = dx / len;
  const double norm_y = dy / len;

  // only draw arrows if it's a unidirectional lane. We used to draw
  // arrows in both directions for bidirectional, but it was messy.
  if (!edge.is_bidirectional())
  {
    const QPen arrow_pen(
      QBrush(QColor::fromRgbF(0.0, 0.0, 0.0, 0.5)),
      lane_pen_width / 8);

    // dimensions for the direction indicators along this path
    const double arrow_w = lane_pen_width / 2.5;  // width of arrowheads
    const double arrow_l = lane_pen_width / 2.5;  // length of arrowheads
    const double arrow_spacing = lane_pen_width * 4.0;

    for (double d = 0.0; d < len; d += arrow_spacing)
    {
      // first calculate the center vertex of this arrowhead
      const double cx = v_start.x + d * norm_x;
      const double cy = v_start.y + d * norm_y;
      // one edge vertex of arrowhead
      const double e1x = cx - arrow_w * norm_y;
      const double e1y = cy + arrow_w * norm_x;
      // another edge vertex of arrowhead
      const double e2x = cx + arrow_w * norm_y;
      const double e2y = cy - arrow_w * norm_x;
      // tip of arrowhead
      const double tx = cx + arrow_l * norm_x;
      const double ty = cy + arrow_l * norm_y;
      // now add arrowhead lines
      scene->addLine(e1x, e1y, tx, ty, arrow_pen);
      scene->addLine(e2x, e2y, tx, ty, arrow_pen);
    }
  }

  QColor color;
  switch (edge.get_graph_idx())
  {
    case 0: color.setRgbF(0.0, 0.5, 0.0); break;
    case 1: color.setRgbF(0.0, 0.0, 0.5); break;
    case 2: color.setRgbF(0.0, 0.5, 0.5); break;
    case 3: color.setRgbF(0.5, 0.5, 0.0); break;
    case 4: color.setRgbF(0.5, 0.0, 0.5); break;
    case 5: color.setRgbF(0.8, 0.0, 0.0); break;
    case 9: color.setRgbF(0.3, 0.3, 0.3); break;
    default: break;  // will render as dark grey
  }

  // always draw lane as red if it's selected
  if (edge.selected)
    color.setRgbF(0.5, 0.0, 0.0);

  // always draw lanes somewhat transparent
  color.setAlphaF(0.5);

  QGraphicsLineItem* lane_item = scene->addLine(
    v_start.x, v_start.y,
    v_end.x, v_end.y,
    QPen(QBrush(color), lane_pen_width, Qt::SolidLine, Qt::RoundCap));
  lane_item->setZValue(edge.get_graph_idx() + 1.0);

  // draw the orientation icon, if specified
  auto orientation_it = edge.params.find("orientation");
  if (orientation_it != edge.params.end())
  {
    // draw robot-outline box midway down this lane
    const double mx = (v_start.x + v_end.x) / 2.0;
    const double my = (v_start.y + v_end.y) / 2.0;
    const double yaw = std::atan2(norm_y, norm_x);

    // robot-box half-dimensions in meters
    const double rw = 0.4 / drawing_meters_per_pixel;
    const double rl = 0.5 / drawing_meters_per_pixel;

    // calculate the corners of the 'robot' box

    // front-left
    // |mx| + |cos -sin| | rl|
    // |my|   |sin  cos| | rw|
    const double flx = mx + rl * std::cos(yaw) - rw * std::sin(yaw);
    const double fly = my + rl * std::sin(yaw) + rw * std::cos(yaw);

    // front-right
    // |mx| + |cos -sin| | rl|
    // |my|   |sin  cos| |-rw|
    const double frx = mx + rl * std::cos(yaw) + rw * std::sin(yaw);
    const double fry = my + rl * std::sin(yaw) - rw * std::cos(yaw);

    // back-left
    // |mx| + |cos -sin| |-rl|
    // |my|   |sin  cos| | rw|
    const double blx = mx - rl * std::cos(yaw) - rw * std::sin(yaw);
    const double bly = my - rl * std::sin(yaw) + rw * std::cos(yaw);

    // back-right
    // |mx| + |cos -sin| |-rl|
    // |my|   |sin  cos| |-rw|
    const double brx = mx - rl * std::cos(yaw) + rw * std::sin(yaw);
    const double bry = my - rl * std::sin(yaw) - rw * std::cos(yaw);

    QPainterPath pp;
    pp.moveTo(QPointF(flx, fly));
    pp.lineTo(QPointF(frx, fry));
    pp.lineTo(QPointF(brx, bry));
    pp.lineTo(QPointF(blx, bly));
    pp.lineTo(QPointF(flx, fly));
    pp.moveTo(QPointF(mx, my));

    QPen orientation_pen(Qt::white, 5.0);
    if (orientation_it->second.value_string == "forward")
    {
      const double hix = mx + 1.0 * cos(yaw) / drawing_meters_per_pixel;
      const double hiy = my + 1.0 * sin(yaw) / drawing_meters_per_pixel;
      pp.lineTo(QPointF(hix, hiy));
      QGraphicsPathItem* pi = scene->addPath(pp, orientation_pen);
      pi->setZValue(edge.get_graph_idx() + 1.1);
    }
    else if (orientation_it->second.value_string == "backward")
    {
      const double hix = mx - 1.0 * cos(yaw) / drawing_meters_per_pixel;
      const double hiy = my - 1.0 * sin(yaw) / drawing_meters_per_pixel;
      pp.lineTo(QPointF(hix, hiy));
      QGraphicsPathItem* pi = scene->addPath(pp, orientation_pen);
      pi->setZValue(edge.get_graph_idx() + 1.1);
    }
  }
}

void Level::draw_wall(QGraphicsScene* scene, const Edge& edge) const
{
  const auto& v_start = vertices[edge.start_idx];
  const auto& v_end = vertices[edge.end_idx];

  const double r = edge.selected ? 0.5 : 0.0;
  const double b = edge.selected ? 0.0 : 0.5;

  scene->addLine(
    v_start.x, v_start.y,
    v_end.x, v_end.y,
    QPen(
      QBrush(QColor::fromRgbF(r, 0.0, b, 0.5)),
      0.2 / drawing_meters_per_pixel,
      Qt::SolidLine, Qt::RoundCap));
}

void Level::draw_meas(QGraphicsScene* scene, const Edge& edge) const
{
  const auto& v_start = vertices[edge.start_idx];
  const auto& v_end = vertices[edge.end_idx];
  const double b = edge.selected ? 0.0 : 0.5;

  scene->addLine(
    v_start.x, v_start.y,
    v_end.x, v_end.y,
    QPen(
      QBrush(QColor::fromRgbF(0.5, 0, b, 0.5)),
      0.5 / drawing_meters_per_pixel,
      Qt::SolidLine, Qt::RoundCap));
}

void Level::draw_door(QGraphicsScene* scene, const Edge& edge) const
{
  const auto& v_start = vertices[edge.start_idx];
  const auto& v_end = vertices[edge.end_idx];
  const double g = edge.selected ? 1.0 : 0.0;
  const double door_thickness = 0.2;  // meters
  const double door_motion_thickness = 0.05;  // meters

  auto door_axis_it = edge.params.find("motion_axis");
  std::string door_axis("start");
  if (door_axis_it != edge.params.end())
    door_axis = door_axis_it->second.value_string;

  double motion_degrees = 90;
  auto motion_degrees_it = edge.params.find("motion_degrees");
  if (motion_degrees_it != edge.params.end())
    motion_degrees = std::abs(motion_degrees_it->second.value_double);

  int motion_dir = 1;
  auto motion_dir_it = edge.params.find("motion_direction");
  if (motion_dir_it != edge.params.end())
    motion_dir = motion_dir_it->second.value_int;

  double right_left_ratio = 1.0;
  auto right_left_ratio_it = edge.params.find("right_left_ratio");
  if (right_left_ratio_it != edge.params.end())
    right_left_ratio = right_left_ratio_it->second.value_double;

  QPainterPath door_motion_path;

  const double door_dx = v_end.x - v_start.x;
  const double door_dy = v_end.y - v_start.y;
  const double door_length = std::sqrt(door_dx * door_dx + door_dy * door_dy);
  const double door_angle = std::atan2(door_dy, door_dx);

  auto door_type_it = edge.params.find("type");
  if (door_type_it != edge.params.end())
  {
    const double DEG2RAD = M_PI / 180.0;

    const std::string& door_type = door_type_it->second.value_string;
    if (door_type == "hinged")
    {
      const double hinge_x = door_axis == "start" ? v_start.x : v_end.x;
      const double hinge_y = door_axis == "start" ? v_start.y : v_end.y;
      const double angle_offset = door_axis == "start" ? 0.0 : M_PI;

      add_door_swing_path(
        door_motion_path,
        hinge_x,
        hinge_y,
        door_length,
        door_angle + angle_offset,
        door_angle + angle_offset + DEG2RAD * motion_dir * motion_degrees);
    }
    else if (door_type == "double_hinged")
    {
      // right door
      add_door_swing_path(
        door_motion_path,
        v_start.x,
        v_start.y,
        (right_left_ratio / (1 + right_left_ratio)) * door_length,
        door_angle,
        door_angle + DEG2RAD * motion_dir * motion_degrees);

      // left door
      add_door_swing_path(
        door_motion_path,
        v_end.x,
        v_end.y,
        (1 / (1 + right_left_ratio)) * door_length,
        door_angle + M_PI,
        door_angle + M_PI - DEG2RAD * motion_dir * motion_degrees);
    }
    else if (door_type == "sliding")
    {
      add_door_slide_path(
        door_motion_path,
        v_start.x,
        v_start.y,
        door_length,
        door_angle);
    }
    else if (door_type == "double_sliding")
    {
      // right door
      add_door_slide_path(
        door_motion_path,
        v_start.x,
        v_start.y,
        (right_left_ratio / (1 + right_left_ratio)) * door_length,
        door_angle);

      // left door
      add_door_slide_path(
        door_motion_path,
        v_end.x,
        v_end.y,
        (1 / (1 + right_left_ratio)) * door_length,
        door_angle + M_PI);
    }
    else
    {
      printf("tried to draw unknown door type: [%s]\n", door_type.c_str());
    }
  }
  scene->addPath(
    door_motion_path,
    QPen(Qt::black, door_motion_thickness / drawing_meters_per_pixel));

  // add the doorjamb last, so it sits on top of the Z stack of the travel arc
  scene->addLine(
    v_start.x, v_start.y,
    v_end.x, v_end.y,
    QPen(
      QBrush(QColor::fromRgbF(1.0, g, 0.0, 0.5)),
      door_thickness / drawing_meters_per_pixel,
      Qt::SolidLine, Qt::RoundCap));
}

void Level::add_door_slide_path(
  QPainterPath& path,
  double hinge_x,
  double hinge_y,
  double door_length,
  double door_angle) const
{
  // first draw the door as a thin line
  path.moveTo(hinge_x, hinge_y);
  path.lineTo(
    hinge_x + door_length * std::cos(door_angle),
    hinge_y + door_length * std::sin(door_angle));

  // now draw a box around where it slides (in the wall, usually)
  const double th = door_angle;  // makes expressions below single-line...
  const double pi_2 = M_PI / 2.0;
  const double s = 0.15 / drawing_meters_per_pixel;  // sliding panel thickness

  const QPointF p1(
    hinge_x - s * std::cos(th + pi_2),
    hinge_y - s * std::sin(th + pi_2));

  const QPointF p2(
    hinge_x - s * std::cos(th + pi_2) - door_length * std::cos(th),
    hinge_y - s * std::sin(th + pi_2) - door_length * std::sin(th));

  const QPointF p3(
    hinge_x + s * std::cos(th + pi_2) - door_length * std::cos(th),
    hinge_y + s * std::sin(th + pi_2) - door_length * std::sin(th));

  const QPointF p4(
    hinge_x + s * std::cos(th + pi_2),
    hinge_y + s * std::sin(th + pi_2));


  path.moveTo(p1);
  path.lineTo(p2);
  path.lineTo(p3);
  path.lineTo(p4);
  path.lineTo(p1);
}

void Level::add_door_swing_path(
  QPainterPath& path,
  double hinge_x,
  double hinge_y,
  double door_length,
  double start_angle,
  double end_angle) const
{
  path.moveTo(hinge_x, hinge_y);
  path.lineTo(
    hinge_x + door_length * std::cos(start_angle),
    hinge_y + door_length * std::sin(start_angle));

  const int NUM_MOTION_STEPS = 10;
  const double angle_inc = (end_angle - start_angle) / (NUM_MOTION_STEPS-1);
  for (int i = 0; i < NUM_MOTION_STEPS; i++)
  {
    // compute door opening angle at this motion step
    const double a = start_angle + i * angle_inc;

    path.lineTo(
      hinge_x + door_length * std::cos(a),
      hinge_y + door_length * std::sin(a));
  }

  path.lineTo(hinge_x, hinge_y);
}

void Level::draw_polygon(
  QGraphicsScene* scene,
  const QBrush& brush,
  const Polygon& polygon) const
{
  QBrush selected_brush(QColor::fromRgbF(1.0, 0.0, 0.0, 0.5));

  QVector<QPointF> polygon_vertices;
  for (const auto& vertex_idx: polygon.vertices)
  {
    const Vertex& v = vertices[vertex_idx];
    polygon_vertices.append(QPointF(v.x, v.y));
  }

  QPen pen(Qt::black);
  pen.setWidthF(0.05 / drawing_meters_per_pixel);

  scene->addPolygon(
    QPolygonF(polygon_vertices),
    pen,
    polygon.selected ? selected_brush : brush);
}

void Level::draw_polygons(QGraphicsScene* scene) const
{
  const QBrush floor_brush(QColor::fromRgbF(0.9, 0.9, 0.9, 0.8));
  const QBrush hole_brush(QColor::fromRgbF(0.3, 0.3, 0.3, 0.5));
  const QBrush roi_brush(QColor::fromRgbF(0.5, 0.5, 0.3, 0.3));

  // first draw the floor polygons
  for (const auto& polygon : polygons)
  {
    if (polygon.type == Polygon::FLOOR)
      draw_polygon(scene, floor_brush, polygon);
  }

  // now draw the holes
  for (const auto& polygon : polygons)
  {
    if (polygon.type == Polygon::HOLE)
      draw_polygon(scene, hole_brush, polygon);
  }

  // now draw the region of interests
  for (const auto& polygon : polygons)
  {
    if (polygon.type == Polygon::ROI)
      draw_polygon(scene, roi_brush, polygon);
  }

#if 0
  // ahhhhh only for debugging...
  // plot the nearest projection point to a polygon, if it's set
  // to something nonzero
  if (level->polygon_edge_proj_x != 0)
  {
    const double r = 5.0;
    addEllipse(
      polygon_edge_proj_x - r,
      polygon_edge_proj_y - r,
      2 * r,
      2 * r,
      QPen(Qt::black),
      QBrush(Qt::blue));
  }
#endif
}

void Level::clear_selection()
{
  for (auto& vertex : vertices)
    vertex.selected = false;

  for (auto& edge : edges)
    edge.selected = false;

  for (auto& model : models)
    model.selected = false;

  for (auto& polygon : polygons)
    polygon.selected = false;

  for (auto& fiducial : fiducials)
    fiducial.selected = false;

  for (auto& feature : floorplan_features)
    feature.setSelected(false);

  for (auto& layer : layers)
    layer.clear_selection();

  for (auto& constraint : constraints)
    constraint.setSelected(false);
}

void Level::draw(
  QGraphicsScene* scene,
  vector<EditorModel>& editor_models,
  const RenderingOptions& rendering_options,
  const vector<Graph>& graphs,
  const CoordinateSystem& coordinate_system)
{
  printf("Level::draw()\n");
  vertex_radius = 0.1;

  if (!coordinate_system.is_global())
  {
    // If we're using an image-defined coordinate system, we should
    // adjust the SceneRect to match the reference image, so the
    // scrollbar range makes sense for this level.
    if (drawing_filename.size() && _drawing_visible)
    {
      const double extra_scroll_area_width = 1.0 * drawing_width;
      const double extra_scroll_area_height = 1.0 * drawing_height;
      scene->setSceneRect(
        QRectF(
          -extra_scroll_area_width,
          -extra_scroll_area_height,
          drawing_width + 2 * extra_scroll_area_width,
          drawing_height + 2 * extra_scroll_area_height));
      scene->addPixmap(floorplan_pixmap);
    }
    else
    {
      const double w = x_meters / drawing_meters_per_pixel;
      const double h = y_meters / drawing_meters_per_pixel;
      scene->setSceneRect(QRectF(0, 0, w, h));
      scene->addRect(0, 0, w, h, Qt::NoPen, Qt::white);
    }
  }

  draw_polygons(scene);

  for (auto& layer : layers)
    layer.draw(scene, drawing_meters_per_pixel, coordinate_system);

  if (rendering_options.show_models)
  {
    for (Model& model : models)
      model.draw(scene, editor_models, drawing_meters_per_pixel);
  }

  for (const auto& edge : edges)
  {
    switch (edge.type)
    {
      case Edge::LANE:
        draw_lane(scene, edge, rendering_options, graphs);
        break;
      case Edge::WALL:
        draw_wall(scene, edge);
        break;
      case Edge::MEAS:
        draw_meas(scene, edge);
        break;
      case Edge::DOOR:
        draw_door(scene, edge);
        break;
      case Edge::HUMAN_LANE:
        draw_lane(scene, edge, rendering_options, graphs);
        break;
      default:
        printf("tried to draw unknown edge type: %d\n",
          static_cast<int>(edge.type));
        break;
    }
  }

  QFont vertex_name_font("Helvetica");
  double vertex_name_font_size =
    vertex_radius / drawing_meters_per_pixel * 1.5 * 10.0;
  if (vertex_name_font_size < 1.0)
    vertex_name_font_size = 1.0;
  vertex_name_font.setPointSizeF(vertex_name_font_size);

  for (const auto& v : vertices)
    v.draw(
      scene,
      vertex_radius / drawing_meters_per_pixel,
      drawing_meters_per_pixel,
      vertex_name_font,
      coordinate_system);

  for (const auto& f : fiducials)
    f.draw(scene, drawing_meters_per_pixel);

  Transform level_scale;
  level_scale.setScale(drawing_meters_per_pixel);
  for (auto& feature : floorplan_features)
  {
    feature.draw(
      scene,
      QColor::fromRgbF(0, 0, 0, 0.5),
      level_scale,
      drawing_meters_per_pixel);
  }

  for (std::size_t i = 0; i < constraints.size(); i++)
    draw_constraint(scene, constraints[i], i);
}

void Level::clear_scene()
{
  for (auto& model : models)
    model.clear_scene();
}

QUuid Level::add_feature(const int layer_idx, const double x, const double y)
{
  if (layer_idx == 0)
  {
    floorplan_features.push_back(Feature(x, y));
    return floorplan_features.rbegin()->id();
  }
  else
  {
    if (layer_idx - 1 >= static_cast<int>(layers.size()))
      return NULL;

    return layers[layer_idx - 1].add_feature(x, y, drawing_meters_per_pixel);
  }
}

void Level::remove_feature(const int layer_idx, QUuid feature_id)
{
  if (layer_idx == 0)
  {
    int index_to_remove = -1;

    for (std::size_t i = 0; i < floorplan_features.size(); i++)
    {
      if (feature_id == floorplan_features[i].id())
        index_to_remove = i;
    }

    if (index_to_remove < 0)
      return;

    floorplan_features.erase(floorplan_features.begin() + index_to_remove);
  }
  else
  {
    if (layer_idx - 1 >= static_cast<int>(layers.size()))
      return;

    layers[layer_idx - 1].remove_feature(feature_id);
  }
}

bool Level::export_features(const std::string& /*filename*/) const
{
  return true;
#if 0
  // TODO (MQ): revisit all of this...

  YAML::Node level_features;

  YAML::Node floorplan_yaml;
  floorplan_yaml["name"] = "floorplan";
  floorplan_yaml["image_file"] = drawing_filename;
  floorplan_yaml["size"].push_back(drawing_width);
  floorplan_yaml["size"].push_back(drawing_height);
  floorplan_yaml["size"].SetStyle(YAML::EmitterStyle::Flow);
  YAML::Node floorplan_cps;
  for (const auto& feature : _feature_sets[0])
  {
    YAML::Node cp_yaml;
    cp_yaml.SetStyle(YAML::EmitterStyle::Flow);
    cp_yaml.push_back(feature.x());
    cp_yaml.push_back(feature.y());
    floorplan_cps.push_back(cp_yaml);
  }
  floorplan_yaml["correspondence_points"] = floorplan_cps;
  level_features["ref_map"] = floorplan_yaml;

  int layer_index = 1;
  for (std::vector<Layer>::const_iterator layer = layers.begin();
    layer != layers.end();
    ++layer)
  {
    YAML::Node y;
    y["name"] = layer->name;
    y["image_file"] = layer->filename;

    QPixmap layer_pixmap = QPixmap::fromImage(
      QImageReader(QString::fromStdString(layer->filename)).read());
    y["size"].push_back(layer_pixmap.width());
    y["size"].push_back(layer_pixmap.height());
    y["size"].SetStyle(YAML::EmitterStyle::Flow);

    YAML::Node transform;
    double scale = layer->meters_per_pixel / drawing_meters_per_pixel;
    transform["scale"].push_back(scale);
    transform["scale"].push_back(scale);
    transform["scale"].SetStyle(YAML::EmitterStyle::Flow);
    transform["rotation"] = layer->rotation;
    transform["translation"].push_back(
      (layer->translation_x / drawing_meters_per_pixel) / scale);
    transform["translation"].push_back(
      (layer->translation_y / drawing_meters_per_pixel) / scale);
    transform["translation"].SetStyle(YAML::EmitterStyle::Flow);
    y["transform"] = transform;

    YAML::Node cps;
    for (const auto& feature : _feature_sets[layer_index])
    {
      QPointF offset = layer->scene_item->offset();
      QPointF mapped_point =
        layer->scene_item->mapFromScene(QPointF(feature.x(), feature.y()));
      YAML::Node cp_yaml;
      cp_yaml.push_back(mapped_point.rx() - offset.rx());
      cp_yaml.push_back(mapped_point.ry() - offset.ry());
      cp_yaml.SetStyle(YAML::EmitterStyle::Flow);
      cps.push_back(cp_yaml);

      //printf("Offset is %f, %f\n", layer->scene_item->offset().rx(), layer->scene_item->offset().ry());
      //QPointF mapped = layer->scene_item->mapFromScene(QPointF(cp.x(), cp.y()));
      //printf("Mapped is %f, %f\n", mapped.rx(), mapped.ry());

      //printf("==================\n");
      //printf("Raw point: %f, %f\n", cp.x(), cp.y());
      //printf("Floorplan scale: %f\n", drawing_meters_per_pixel);
      //printf("Translation in metres: %f, %f\n", layer->translation_x, layer->translation_y);

      //double scale = layer->meters_per_pixel / drawing_meters_per_pixel;
      //double displayed_image_x = layer->pixmap.width() * scale;
      //double displayed_image_y = layer->pixmap.height() * scale;
      //printf("Image size: %f, %f\n", displayed_image_x, displayed_image_y);
      //printf("------------------\n");
      //double image_x = cp.x();
      //double image_y = cp.y();

      //double translate_x = (layer->translation_x / drawing_meters_per_pixel);
      //double translate_y = (layer->translation_y / drawing_meters_per_pixel);
      //image_x = image_x - translate_x;
      //image_y = image_y + translate_y;
      //printf("Translated x by %f to %f\n", translate_x, image_x);
      //printf("Translated y by %f to %f\n", translate_y, image_y);

      //double angle = -layer->rotation;
      //image_x = image_x * std::cos(angle) - image_y * std::sin(angle);
      //image_y = image_x * std::sin(angle) + image_y * std::cos(angle);
      //printf("Rotated x by %f to %f\n", angle, image_x);
      //printf("Rotated y by %f to %f\n", angle, image_y);

      //image_x = image_x / scale;
      //image_y = image_y / scale;
      //printf("Scaled x by %f to %f\n", scale, image_x);
      //printf("Scaled y by %f to %f\n", scale, image_y);

      //printf("Offset is %f, %f\n", layer->scene_item->offset().rx(), layer->scene_item->offset().ry());
      //QPointF mapped = layer->scene_item->mapFromScene(QPointF(cp.x(), cp.y()));
      //printf("Mapped is %f, %f\n", mapped.rx(), mapped.ry());

    }
    y["correspondence_points"] = cps;
    layer_index++;
    level_features["robot_map"] = y;
  }

  YAML::Emitter emitter;
  yaml_utils::write_node(level_features, emitter);
  std::ofstream dest(filename);
  dest << emitter.c_str();
  return true;
#endif
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

  for (std::size_t v0_idx = 0; v0_idx < polygon->vertices.size(); v0_idx++)
  {
    const std::size_t v1_idx =
      v0_idx < polygon->vertices.size() - 1 ? v0_idx + 1 : 0;
    const std::size_t v0 = polygon->vertices[v0_idx];
    const std::size_t v1 = polygon->vertices[v1_idx];

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
  for (std::size_t i = 0; i < polygon->vertices.size(); i++)
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

void Level::add_vertex(const double x, const double y)
{
  vertices.push_back(Vertex(x, y));
}

std::size_t Level::get_vertex_by_id(QUuid vertex_id)
{
  for (std::size_t i = 0; i < vertices.size(); i++)
  {
    if (vertices[i].uuid == vertex_id)
    {
      return i;
    }
  }
  return vertices.size()+1;
}

bool Level::are_layer_names_unique()
{
  // Just do the trivial n^2 approach for now, if we ever have a zillion
  // layers, we can do something more sophisticated.

  for (std::size_t i = 0; i < layers.size(); i++)
  {
    for (std::size_t j = i + 1; j < layers.size(); j++)
    {
      if (layers[i].name == layers[j].name)
      {
        printf("layer %d (%s) is the same as layer %d (%s)\n",
          static_cast<int>(i),
          layers[i].name.c_str(),
          static_cast<int>(j),
          layers[j].name.c_str());

        return false;
      }
    }
  }

  return true;
}

const Feature* Level::find_feature(const QUuid& id) const
{
  for (std::size_t i = 0; i < floorplan_features.size(); i++)
  {
    if (floorplan_features[i].id() == id)
      return &floorplan_features[i];
  }
  for (std::size_t layer_idx = 0; layer_idx < layers.size(); layer_idx++)
  {
    for (std::size_t i = 0; i < layers[layer_idx].features.size(); i++)
    {
      if (layers[layer_idx].features[i].id() == id)
        return &layers[layer_idx].features[i];
    }
  }
  return nullptr;
}

const Feature* Level::find_feature(const double x, const double y) const
{
  for (std::size_t layer_idx = 0; layer_idx < layers.size(); layer_idx++)
  {
    if (!layers[layer_idx].visible)
      continue;

    const Feature* layer_feature =
      layers[layer_idx].find_feature(x, y, drawing_meters_per_pixel);

    if (layer_feature)
      return layer_feature;
  }

  double min_dist = 1e9;
  const Feature* min_feature = nullptr;

  for (std::size_t i = 0; i < floorplan_features.size(); i++)
  {
    const Feature& f = floorplan_features[i];
    const double dx = x - f.x();
    const double dy = y - f.y();
    const double dist = sqrt(dx * dx + dy * dy);
    if (dist < min_dist)
    {
      min_dist = dist;
      min_feature = &floorplan_features[i];
    }
  }
  if (min_dist < Feature::radius_meters / drawing_meters_per_pixel)
    return min_feature;

  return nullptr;
}

void Level::add_constraint(const QUuid& a, const QUuid& b)
{
  printf("Level::add_constraint(%s, %s)\n",
    a.toString().toStdString().c_str(),
    b.toString().toStdString().c_str());
  if (a == b)
    return;
  constraints.push_back(Constraint(a, b));
}

void Level::remove_constraint(const QUuid& a, const QUuid& b)
{
  const Constraint c(a, b);
  int index_to_remove = -1;

  for (std::size_t i = 0; i < constraints.size(); i++)
  {
    if (constraints[i] == c)
    {
      index_to_remove = static_cast<int>(i);
      break;
    }
  }

  if (index_to_remove < 0)
    return;

  constraints.erase(constraints.begin() + index_to_remove);
}

bool Level::get_feature_point(const QUuid& id, QPointF& point) const
{
  for (std::size_t i = 0; i < floorplan_features.size(); i++)
  {
    if (floorplan_features[i].id() == id)
    {
      point = floorplan_features[i].qpoint();
      return true;
    }
  }
  for (std::size_t layer_idx = 0; layer_idx < layers.size(); layer_idx++)
  {
    for (std::size_t i = 0; i < layers[layer_idx].features.size(); i++)
    {
      const Layer& layer = layers[layer_idx];
      if (layer.features[i].id() == id)
      {
        const Feature& feature = layers[layer_idx].features[i];
        point = layer.transform.forwards(feature.qpoint());
        point /= drawing_meters_per_pixel;
        return true;
      }
    }
  }
  return false;
}

void Level::draw_constraint(
  QGraphicsScene* scene,
  const Constraint& constraint,
  int constraint_idx) const
{
  const std::vector<QUuid>& feature_ids = constraint.ids();
  if (feature_ids.size() != 2)
  {
    printf("WOAH! tried to draw a constraint with only %d ID's!\n",
      static_cast<int>(feature_ids.size()));
    return;
  }

  const QColor color = QColor::fromRgbF(0.7, 0.7, 0.2, 1.0);
  const QColor selected_color = QColor::fromRgbF(1.0, 0.0, 0.0, 0.5);

  const double pen_width = 0.1 / drawing_meters_per_pixel;
  QPen pen(
    QBrush(constraint.selected() ? selected_color : color),
    pen_width,
    Qt::SolidLine,
    Qt::RoundCap);

  QPointF p1, p2;
  if (!get_feature_point(feature_ids[0], p1))
  {
    printf("woah! couldn't find constraint feature ID %s\n",
      feature_ids[0].toString().toStdString().c_str());
    return;
  }

  if (!get_feature_point(feature_ids[1], p2))
  {
    printf("woah! couldn't find constraint feature ID %s\n",
      feature_ids[1].toString().toStdString().c_str());
    return;
  }

  QGraphicsLineItem* line = scene->addLine(
    p1.x(),
    p1.y(),
    p2.x(),
    p2.y(),
    pen);
  line->setZValue(199.0);
  line->setData(0, "constraint");
  line->setData(1, constraint_idx);
}

class TransformResidual
{
public:
  TransformResidual(
    double level_x,
    double level_y,
    double level_meters_per_pixel,
    double layer_x,
    double layer_y)
  : _level_x(level_x),
    _level_y(level_y),
    _level_meters_per_pixel(level_meters_per_pixel),
    _layer_x(layer_x),
    _layer_y(layer_y)
  {
  }

  template<typename T>
  bool operator()(
    const T* const yaw,
    const T* const scale,
    const T* const translation,
    T* residual) const
  {
    const T qx =
      (( cos(yaw[0]) * _layer_x + sin(yaw[0]) * _layer_y) * scale[0]
      + translation[0]) / _level_meters_per_pixel;

    const T qy =
      ((-sin(yaw[0]) * _layer_x + cos(yaw[0]) * _layer_y) * scale[0]
      + translation[1]) / _level_meters_per_pixel;

    residual[0] = _level_x - qx;
    residual[1] = _level_y - qy;

    return true;
  }

private:
  double _level_x, _level_y;
  double _level_meters_per_pixel;
  double _layer_x, _layer_y;
};

void Level::optimize_layer_transforms()
{
  printf("level %s optimizing layer transforms...\n", name.c_str());

  for (std::size_t i = 0; i < layers.size(); i++)
  {
    ceres::Problem problem;

    double yaw = layers[i].transform.yaw();
    double scale = layers[i].transform.scale();
    double translation[2] = {
      layers[i].transform.translation().x(),
      layers[i].transform.translation().y()
    };
    int num_constraints_found = 0;

    for (const Constraint& constraint : constraints)
    {
      const std::vector<QUuid>& feature_ids = constraint.ids();
      if (feature_ids.size() != 2)
        continue;

      QPointF level_point, layer_point;

      // find the level point
      bool found_level_point = false;
      for (const Feature& feature : floorplan_features)
      {
        if (feature_ids[0] == feature.id())
        {
          level_point = feature.qpoint();
          found_level_point = true;
          break;
        }
        else if (feature_ids[1] == feature.id())
        {
          level_point = feature.qpoint();
          found_level_point = true;
          break;
        }
      }
      if (!found_level_point)
      {
        printf("WOAH couldn't find level point in constraint! Ignoring it\n");
        continue;
      }

      // find the layer point
      bool found_layer_point = false;
      for (const Feature& feature : layers[i].features)
      {
        if (feature_ids[0] == feature.id())
        {
          layer_point = feature.qpoint();
          found_layer_point = true;
          break;
        }
        else if (feature_ids[1] == feature.id())
        {
          layer_point = feature.qpoint();
          found_layer_point = true;
          break;
        }
      }
      if (!found_layer_point)
      {
        printf("WOAH couldn't find layer point in constraint! Ignoring...\n");
        continue;
      }

      TransformResidual* tr = new TransformResidual(
        level_point.x(),
        level_point.y(),
        drawing_meters_per_pixel,
        layer_point.x(),
        layer_point.y());

      problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<TransformResidual, 2, 1, 1, 2>(tr),
        nullptr,
        &yaw,
        &scale,
        &translation[0]);

      ++num_constraints_found;
    }

    // Make sure there were features for this layer, otherwise ceres will fail
    if (num_constraints_found == 0)
    {
      printf("No constraints found for layer, skipping optimization\n");
      continue;
    }
    problem.SetParameterLowerBound(&scale, 0, 0.01);

    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    printf("solution:\n");
    printf("  yaw = %.3f\n", yaw);
    printf("  scale = %.3f\n", scale);
    printf("  translation = (%.3f, %.3f)\n", translation[0], translation[1]);

    layers[i].transform.setYaw(yaw);
    layers[i].transform.setScale(scale);
    layers[i].transform.setTranslation(
      QPointF(translation[0], translation[1]));
  }
}

void Level::mouse_select_press(
  const double x,
  const double y,
  QGraphicsItem* graphics_item,
  const RenderingOptions& rendering_options,
  const Qt::KeyboardModifiers& modifiers)
{
  printf("Level::mouse_select_press(%.3f, %.3f)\n", x, y);

  if (!(modifiers & Qt::ShiftModifier))
    clear_selection();

  const NearestItem ni = nearest_items(x, y);

  const double vertex_dist_thresh = vertex_radius / drawing_meters_per_pixel;

  const double feature_dist_thresh =
    Feature::radius_meters / drawing_meters_per_pixel;

  // todo: use QGraphics stuff to see if we clicked a model pixmap...
  const double model_dist_thresh = 0.5 / drawing_meters_per_pixel;

  if (rendering_options.show_models &&
    ni.model_idx >= 0 &&
    ni.model_dist < model_dist_thresh)
    models[ni.model_idx].selected = true;
  else if (ni.vertex_idx >= 0 && ni.vertex_dist < vertex_dist_thresh)
    vertices[ni.vertex_idx].selected = true;
  else if (ni.feature_idx >= 0 && ni.feature_dist < feature_dist_thresh)
  {
    //levels[level_idx].feature_sets[
    printf("feature_layer_idx = %d, feature_idx = %d, feature_dist = %.3f\n",
      ni.feature_layer_idx,
      ni.feature_idx,
      ni.feature_dist);

    if (ni.feature_layer_idx == 0)
    {
      floorplan_features[ni.feature_idx].setSelected(true);
    }
    else
    {
      layers[ni.feature_layer_idx-1].
      features[ni.feature_idx].setSelected(true);
    }
  }
  else if (ni.fiducial_idx >= 0 && ni.fiducial_dist < 10.0)
    fiducials[ni.fiducial_idx].selected = true;
  else
  {
    // use the QGraphics stuff to see if it's an edge segment or polygon
    if (graphics_item)
    {
      switch (graphics_item->type())
      {
        case QGraphicsLineItem::Type:
          set_selected_line_item(
            qgraphicsitem_cast<QGraphicsLineItem*>(graphics_item),
            rendering_options);
          break;

        case QGraphicsPolygonItem::Type:
          set_selected_containing_polygon(x, y);
          break;

        default:
          printf("clicked unhandled type: %d\n",
            static_cast<int>(graphics_item->type()));
          break;
      }
    }
  }
}

Level::NearestItem Level::nearest_items(const double x, const double y)
{
  NearestItem ni;

  for (std::size_t i = 0; i < vertices.size(); i++)
  {
    const Vertex& p = vertices[i];
    const double dx = x - p.x;
    const double dy = y - p.y;
    const double dist = sqrt(dx*dx + dy*dy);
    if (dist < ni.vertex_dist)
    {
      ni.vertex_dist = dist;
      ni.vertex_idx = i;
    }
  }

  // search the floorplan features
  for (std::size_t i = 0; i < floorplan_features.size(); i++)
  {
    const Feature& f = floorplan_features[i];
    const double dx = x - f.x();
    const double dy = y - f.y();
    const double dist = sqrt(dx*dx + dy*dy);
    if (dist < ni.feature_dist)
    {
      ni.feature_layer_idx = 0;
      ni.feature_dist = dist;
      ni.feature_idx = i;
    }
  }

  // now search all "other" layer features
  for (std::size_t layer_idx = 0; layer_idx < layers.size(); layer_idx++)
  {
    const Layer& layer = layers[layer_idx];
    for (std::size_t i = 0; i < layer.features.size(); i++)
    {
      const Feature& f = layer.features[i];

      // transform this point into parent level's pixel space
      QPointF p(layer.transform.forwards(f.qpoint()));
      p /= drawing_meters_per_pixel;

      const double dx = x - p.x();
      const double dy = y - p.y();
      const double dist = sqrt(dx*dx + dy*dy);
      if (dist < ni.feature_dist)
      {
        ni.feature_layer_idx = layer_idx + 1;
        ni.feature_dist = dist;
        ni.feature_idx = i;
      }
    }
  }

  for (std::size_t i = 0; i < fiducials.size(); i++)
  {
    const Fiducial& f = fiducials[i];
    const double dx = x - f.x;
    const double dy = y - f.y;
    const double dist = sqrt(dx*dx + dy*dy);
    if (dist < ni.fiducial_dist)
    {
      ni.fiducial_dist = dist;
      ni.fiducial_idx = i;
    }
  }

  for (std::size_t i = 0; i < models.size(); i++)
  {
    const Model& m = models[i];
    const double dx = x - m.state.x;
    const double dy = y - m.state.y;
    const double dist = sqrt(dx*dx + dy*dy);  // no need for sqrt each time
    if (dist < ni.model_dist)
    {
      ni.model_dist = dist;
      ni.model_idx = i;
    }
  }

  return ni;
}

int Level::nearest_item_index_if_within_distance(
  const double x,
  const double y,
  const double distance_threshold,
  const ItemType item_type)
{
  double min_dist = 1e100;
  int min_index = -1;
  if (item_type == VERTEX)
  {
    for (std::size_t i = 0; i < vertices.size(); i++)
    {
      const Vertex& p = vertices[i];
      const double dx = x - p.x;
      const double dy = y - p.y;
      const double dist2 = dx*dx + dy*dy;  // no need for sqrt each time
      if (dist2 < min_dist)
      {
        min_dist = dist2;
        min_index = i;
      }
    }
  }
  else if (item_type == FIDUCIAL)
  {
    for (std::size_t i = 0; i < fiducials.size(); i++)
    {
      const Fiducial& f = fiducials[i];
      const double dx = x - f.x;
      const double dy = y - f.y;
      const double dist2 = dx*dx + dy*dy;
      if (dist2 < min_dist)
      {
        min_dist = dist2;
        min_index = i;
      }
    }
  }
  else if (item_type == MODEL)
  {
    for (std::size_t i = 0; i < models.size(); i++)
    {
      const Model& m = models[i];
      const double dx = x - m.state.x;
      const double dy = y - m.state.y;
      const double dist2 = dx*dx + dy*dy;  // no need for sqrt each time
      if (dist2 < min_dist)
      {
        min_dist = dist2;
        min_index = i;
      }
    }
  }
  if (sqrt(min_dist) < distance_threshold)
    return min_index;
  return -1;
}

void Level::set_selected_line_item(
  QGraphicsLineItem* line_item,
  const RenderingOptions& rendering_options)
{
  clear_selection();
  if (line_item == nullptr)
    return;

  // look up the line's vertices
  const double x1 = line_item->line().x1();
  const double y1 = line_item->line().y1();
  const double x2 = line_item->line().x2();
  const double y2 = line_item->line().y2();

  double min_edge_dist = 1e9;
  Edge* min_edge = nullptr;
  // find if any of our lanes match those vertices
  for (auto& edge : edges)
  {
    if ((edge.type == Edge::LANE) &&
      (edge.get_graph_idx() != rendering_options.active_traffic_map_idx))
      continue;

    const auto& v_start = vertices[edge.start_idx];
    const auto& v_end = vertices[edge.end_idx];

    // calculate distances
    const double dx1 = v_start.x - x1;
    const double dy1 = v_start.y - y1;
    const double dx2 = v_end.x - x2;
    const double dy2 = v_end.y - y2;
    const double v1_dist = std::sqrt(dx1*dx1 + dy1*dy1);
    const double v2_dist = std::sqrt(dx2*dx2 + dy2*dy2);

    const double max_dist = (v1_dist > v2_dist ? v1_dist : v2_dist);
    if (max_dist < min_edge_dist)
    {
      min_edge_dist = max_dist;
      min_edge = &edge;
    }
  }

  const double thresh = 10.0;  // should be really tiny if it matches
  if (min_edge_dist < thresh && min_edge != nullptr)
  {
    min_edge->selected = true;
    return;
  }

  // see if the constraint index is stored in the QGraphicsItem
  if (line_item->data(0).toString() == "constraint")
  {
    if (line_item->data(1).isValid())
    {
      const int constraint_idx = line_item->data(1).toInt();
      printf("constraint index: %d\n", constraint_idx);
      if (constraint_idx >= 0 &&
        constraint_idx < static_cast<int>(constraints.size()))
      {
        constraints[constraint_idx].setSelected(true);
      }
      return;
    }
  }
}

void Level::set_selected_containing_polygon(
  const double x,
  const double y)
{
  // holes are "higher" in our Z-stack (to make them clickable), so first
  // we need to make a list of all polygons that contain this point.
  vector<Polygon*> containing_polygons;
  for (std::size_t i = 0; i < polygons.size(); i++)
  {
    Polygon& polygon = polygons[i];
    QVector<QPointF> polygon_vertices;
    for (const auto& vertex_idx: polygon.vertices)
    {
      const Vertex& v = vertices[vertex_idx];
      polygon_vertices.append(QPointF(v.x, v.y));
    }
    QPolygonF qpolygon(polygon_vertices);
    if (qpolygon.containsPoint(QPoint(x, y), Qt::OddEvenFill))
      containing_polygons.push_back(&polygons[i]);
  }

  // first search for holes
  for (Polygon* p : containing_polygons)
  {
    if (p->type == Polygon::HOLE)
    {
      p->selected = true;
      return;
    }
  }

  // if we get here, just return the first thing.
  for (Polygon* p : containing_polygons)
  {
    p->selected = true;
    return;
  }
}

void Level::compute_layer_transforms()
{
  printf("Level::compute_layer_transforms()\n");
  for (std::size_t i = 0; i < layers.size(); i++)
    compute_layer_transform(i);
}

void Level::compute_layer_transform(const std::size_t layer_idx)
{
  printf("Level::compute_layer_transform(%d)\n", static_cast<int>(layer_idx));
  if (layer_idx >= layers.size())
    return;
  Layer& layer = layers[layer_idx];

  layer.transform_strings.clear();

  const double ff_rmf_scale = 0.05;  // standard 5cm grid cell size
  Transform ff_rmf;
  ff_rmf.setScale(ff_rmf_scale / layer.transform.scale());

  const double ff_map_height = ff_rmf_scale * layer.image.height();

  ff_rmf.setYaw(-(fmod(layer.transform.yaw() + M_PI, 2 * M_PI) - M_PI));

  const double tx =
    -(layer.transform.translation().x() -
    ff_map_height / ff_rmf.scale() * sin(ff_rmf.yaw())) *
    ff_rmf.scale();

  const double ty =
    (layer.transform.translation().y() +
    ff_map_height / ff_rmf.scale() * cos(ff_rmf.yaw())) *
    ff_rmf.scale();

  // FreeFleet does its translation first, so... we have to rotate this
  // translation vector and scale it into the FreeFleet robot's frame

  const double yaw = ff_rmf.yaw();
  ff_rmf.setTranslation(
    QPointF(
      cos(-yaw) * tx + sin(-yaw) * ty,
      -sin(-yaw) * tx + cos(-yaw) * ty));

  printf("tx = %.5f ty = %.5f mh = %.5f sy = %.5f cy = %.5f\n",
    layer.transform.translation().x(),
    layer.transform.translation().y(),
    ff_map_height,
    sin(ff_rmf.yaw()),
    cos(ff_rmf.yaw()));

  layer.transform_strings.push_back(
    std::make_pair(
      "5cm FreeFleet -> RMF\ntranslate, rotate, scale",
      ff_rmf.to_string()));

  Transform gridcells_rmf;
  gridcells_rmf.setScale(layer.transform.scale());
  gridcells_rmf.setYaw(fmod(layer.transform.yaw() + M_PI, 2 * M_PI) - M_PI);
  const double gx =
    (layer.transform.translation().x() +
    layer.image.height() * gridcells_rmf.scale() * sin(gridcells_rmf.yaw()));
  const double gy =
    (layer.transform.translation().y() +
    layer.image.height() * gridcells_rmf.scale() * cos(gridcells_rmf.yaw()));
  gridcells_rmf.setTranslation(QPointF(gx, gy));

  layer.transform_strings.push_back(
    std::make_pair(
      "grid cells -> RMF\nscale, rotate, translate",
      gridcells_rmf.to_string()));

  layer.transform_strings.push_back(
    std::make_pair(
      "RMF -> grid cells\nscale, rotate, translate",
      gridcells_rmf.inverse().to_string()));
}

void Level::align_colinear()
{
  struct SelectedVertex
  {
    size_t index;
    bool expanded;
    vector<size_t> connected_vertex_indices;
  };

  // build up a vector of selected vertex indices
  vector<SelectedVertex> selected_vertices;
  for (size_t i = 0; i < vertices.size(); i++)
  {
    if (vertices[i].selected)
    {
      SelectedVertex sv;
      sv.index = i;
      sv.expanded = false;

      for (size_t j = 0; j < edges.size(); j++)
      {
        const size_t start_idx = static_cast<size_t>(edges[j].start_idx);
        const size_t end_idx = static_cast<size_t>(edges[j].end_idx);
        if (start_idx == i && vertices[end_idx].selected)
          sv.connected_vertex_indices.push_back(end_idx);
        else if (end_idx == i && vertices[start_idx].selected)
          sv.connected_vertex_indices.push_back(start_idx);
      }

      selected_vertices.push_back(sv);
    }
  }

  printf("align_colinear() vertices:\n");
  for (const SelectedVertex& sv : selected_vertices)
  {
    printf("  %zu:", sv.index);
    for (const size_t i : sv.connected_vertex_indices)
      printf(" %zu", i);
    printf("\n");
  }

  if (selected_vertices.size() < 3)
  {
    printf("%zu vertices were selected; >= 3 required for colinear align!\n",
      selected_vertices.size());
    return;
  }

  // start at one endpoint and go down the chain
  vector<SelectedVertex> chain;
  for (size_t i = 0; i < selected_vertices.size(); i++)
  {
    if (selected_vertices[i].connected_vertex_indices.size() == 1)
    {
      chain.push_back(selected_vertices[i]);
      break;
    }
  }

  if (chain.empty())
  {
    printf("could not find starting point of chain; I'll just make a guess\n");
    // for now, just use the element with the smallest horizontal coordinate
    // to be more fancy in the future, we could make a regression line and
    // choose the point closest to its end
    size_t min_idx = 0;
    double min_value = 1e42;
    for (size_t i = 0; i < selected_vertices.size(); i++)
    {
      const Vertex& v = vertices[selected_vertices[i].index];
      if (v.x < min_value)
      {
        min_value = v.x;
        min_idx = i;
      }
    }
    chain.push_back(selected_vertices[min_idx]);
  }

  // keep expanding the last endpoint
  while (true)
  {
    bool chain_complete = true;
    for (size_t i = 0; i < chain.back().connected_vertex_indices.size(); i++)
    {
      const size_t test_vertex_idx = chain.back().connected_vertex_indices[i];
      // see if this is a new vertex to add to the chain
      bool found = false;
      for (size_t j = 0; j < chain.size() - 1; j++)
      {
        if (chain[j].index == test_vertex_idx)
        {
          found = true;
          break;
        }
      }

      if (!found)
      {
        printf("  adding vertex %zu to chain\n", test_vertex_idx);
        for (size_t j = 0; j < selected_vertices.size(); j++)
        {
          if (selected_vertices[j].index == test_vertex_idx)
            chain.push_back(selected_vertices[j]);
        }
        chain_complete = false;
      }
    }

    if (chain_complete)
      break;
  }

  printf("  chain before sorting:");
  for (const SelectedVertex& sv : chain)
    printf(" %zu", sv.index);
  printf("\n");

  // sort the chain vertices by distance from the starting vertex
  const SelectedVertex starting_vertex = chain[0];
  std::sort(
    chain.begin() + 1,
    chain.end(),
    [this, starting_vertex](SelectedVertex sv1, SelectedVertex sv2)
    {
      const Vertex& v = vertices[starting_vertex.index];
      const Vertex& v1 = vertices[sv1.index];
      const Vertex& v2 = vertices[sv2.index];

      const double d1x = v.x - v1.x;
      const double d1y = v.y - v1.y;
      const double dist_v1 = sqrt(d1x * d1x + d1y * d1y);

      const double d2x = v.x - v2.x;
      const double d2y = v.y - v2.y;
      const double dist_v2 = sqrt(d2x * d2x + d2y * d2y);
      return dist_v1 < dist_v2;
    });

  printf("  chain:");
  for (const SelectedVertex& sv : chain)
    printf(" %zu", sv.index);
  printf("\n");

  if (chain.size() < 3)
  {
    printf("could not find a connected chain of >= 3 vertices!\n");
    return;
  }

  // compute line between first and last vertices
  const Vertex& v1 = vertices[chain.front().index];
  const Vertex& v2 = vertices[chain.back().index];

  // compute unit vector pointing from v1 to v2
  const double line_length =
    sqrt((v2.x - v1.x) * (v2.x - v1.x) + (v2.y - v1.y) * (v2.y - v1.y));
  if (line_length < 0.001)
  {
    printf("ill-defined line! bailing to avoid numerical blowups\n");
    return;
  }
  const double ux = (v2.x - v1.x) / line_length;
  const double uy = (v2.y - v1.y) / line_length;

  printf("line: (%.3f, %.3f), (%.3f, %.3f)  u = (%.3f, %.3f)\n",
    v1.x, v1.y, v2.x, v2.y, ux, uy);

  // project intermediate vertices onto this line
  for (size_t i = 1; i < chain.size() - 1; i++)
  {
    Vertex& v = vertices[chain[i].index];
    const double t = ((v1.x - v.x) * ux) + ((v1.y - v.y) * uy);
    v.x = v1.x - t * ux;
    v.y = v1.y - t * uy;
  }
}
