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

#include <cmath>

#include <QGraphicsScene>
#include <QGraphicsSimpleTextItem>
#include <QIcon>

#include "vertex.h"
using std::string;
using std::vector;
using std::pair;

const vector<pair<string, Param::Type>> Vertex::allowed_params
{
  { "is_parking_spot", Param::Type::BOOL },
  { "is_charger", Param::Type::BOOL},
  { "dock_name", Param::Type::STRING},
  { "is_cleaning_zone", Param::Type::BOOL},
  { "dropoff_ingestor", Param::Type::STRING },
  { "pickup_dispenser", Param::Type::STRING },
  { "spawn_robot_type", Param::Type::STRING },
  { "spawn_robot_name", Param::Type::STRING },
  { "is_holding_point", Param::Type::BOOL },
  { "is_passthrough_point", Param::Type::BOOL },
  { "human_goal_set_name", Param::Type::STRING },
};


Vertex::Vertex()
: x(0), y(0), selected(false)
{
  uuid = QUuid::createUuid();
}

Vertex::Vertex(double _x, double _y, const string& _name)
: x(_x), y(_y), name(_name), selected(false)
{
  uuid = QUuid::createUuid();
}

void Vertex::from_yaml(
  const YAML::Node& data,
  const CoordinateSystem& coordinate_system)
{
  if (!data.IsSequence())
    throw std::runtime_error("Vertex::from_yaml expected a sequence");

  if (!coordinate_system.is_global())
  {
    x = data[0].as<double>();
    y = data[1].as<double>();
  }
  else
  {
    CoordinateSystem::WGS84Point wgs84_point;
    wgs84_point.lon = data[0].as<double>();
    wgs84_point.lat = data[1].as<double>();

    CoordinateSystem::ProjectedPoint p =
      coordinate_system.to_epsg3857(wgs84_point);
    x = p.x;
    y = p.y;
  }

  if (data.size() < 4)
    return;// todo: remove... intended only during format transition
  // skip the z-offset in data[2] for now
  name = data[3].as<string>();

  // load the parameters, all of which (including the params map) are
  // optional at the moment.
  if (data.size() >= 4)
  {
    for (YAML::const_iterator it = data[4].begin(); it != data[4].end(); ++it)
    {
      Param p;
      p.from_yaml(it->second);
      params[it->first.as<string>()] = p;
    }
  }
}

YAML::Node Vertex::to_yaml(const CoordinateSystem& coordinate_system) const
{
  YAML::Node vertex_node;
  vertex_node.SetStyle(YAML::EmitterStyle::Flow);

  if (!coordinate_system.is_global())
  {
    // in either image or cartesian-meters coordinate spaces, we're
    // fine with rounding to 3 decimal places
    vertex_node.push_back(std::round(x * 1000.0) / 1000.0);
    vertex_node.push_back(std::round(y * 1000.0) / 1000.0);
  }
  else
  {
    // convert back to WGS84 and save with as many decimal places as possible
    CoordinateSystem::WGS84Point p = coordinate_system.to_wgs84({x, y});
    vertex_node.push_back(p.lon);
    vertex_node.push_back(p.lat);
  }
  vertex_node.push_back(0.0);  // placeholder for Z offsets in the future
  vertex_node.push_back(name);

  if (!params.empty())
  {
    YAML::Node params_node(YAML::NodeType::Map);
    for (const auto& param : params)
      params_node[param.first] = param.second.to_yaml();
    vertex_node.push_back(params_node);
  }
  return vertex_node;
}

void Vertex::draw(
  QGraphicsScene* scene,
  const double radius,
  const QFont& font,
  const CoordinateSystem& coordinate_system) const
{
  QPen vertex_pen(Qt::black);
  vertex_pen.setWidthF(radius / 2.0);

  const double a = 0.5;

  const QColor vertex_color = QColor::fromRgbF(0.0, 0.5, 0.0);
  QColor nonselected_color(vertex_color);
  nonselected_color.setAlphaF(a);

  QColor selected_color = QColor::fromRgbF(1.0, 0.0, 0.0, a);

  const QBrush vertex_brush =
    selected ? QBrush(selected_color) : QBrush(nonselected_color);

  QGraphicsEllipseItem* ellipse_item = scene->addEllipse(
    x - radius,
    y - radius,
    2 * radius,
    2 * radius,
    vertex_pen,
    vertex_brush);
  ellipse_item->setZValue(20.0);  // above all lane/wall edges

  // add some icons depending on the superpowers of this vertex
  QPen annotation_pen(Qt::black);
  annotation_pen.setWidthF(radius / 4.0);
  const double icon_ring_radius = radius * 2.5;
  const double icon_scale = 2.0 * radius / 128.0;

  if (is_holding_point())
  {
    const double icon_bearing = -135.0 * M_PI / 180.0;
    QIcon icon(":icons/stopwatch.svg");
    QPixmap pixmap(icon.pixmap(icon.actualSize(QSize(128, 128))));
    QGraphicsPixmapItem* pixmap_item = scene->addPixmap(pixmap);
    pixmap_item->setOffset(
      -pixmap.width() / 2,
      -pixmap.height() / 2);
    pixmap_item->setScale(icon_scale);
    pixmap_item->setZValue(20.0);
    pixmap_item->setPos(
      x + icon_ring_radius * cos(icon_bearing),
      y - icon_ring_radius * sin(icon_bearing));
    if (!coordinate_system.is_y_flipped())
      pixmap_item->setTransform(pixmap_item->transform().scale(1, -1));
    pixmap_item->setToolTip("This vertex is a holding point");
  }

  if (is_parking_point())
  {
    const double icon_bearing = 45.0 * M_PI / 180.0;
    QIcon icon(":icons/parking.svg");
    QPixmap pixmap(icon.pixmap(icon.actualSize(QSize(128, 128))));
    QGraphicsPixmapItem* pixmap_item = scene->addPixmap(pixmap);
    pixmap_item->setOffset(
      -pixmap.width() / 2,
      -pixmap.height() / 2);
    pixmap_item->setScale(icon_scale);
    pixmap_item->setZValue(20.0);
    pixmap_item->setPos(
      x + icon_ring_radius * cos(icon_bearing),
      y - icon_ring_radius * sin(icon_bearing));
    if (!coordinate_system.is_y_flipped())
      pixmap_item->setTransform(pixmap_item->transform().scale(1, -1));
    pixmap_item->setToolTip("This vertex is a parking point");

    /*
    // outline the vertex with another circle
    QGraphicsEllipseItem* ellipse_item = scene->addEllipse(
      x - 2.0 * radius,
      y - 2.0 * radius,
      2 * 2.0 * radius,
      2 * 2.0 * radius,
      annotation_pen,
      QBrush());  // default brush is transparent
    ellipse_item->setZValue(20.0);  // above all lane/wall edges
    */
  }

  if (is_charger())
  {
    const double icon_bearing = 135.0 * M_PI / 180.0;
    /*
    // draw a larger black rectangle around the vertex
    QGraphicsRectItem* rect_item = scene->addRect(
      x - 2.0 * radius,
      y - 2.0 * radius,
      2 * 2.0 * radius,
      2 * 2.0 * radius,
      annotation_pen,
      QBrush());  // default brush is transparent
    rect_item->setZValue(20.0);
    */
    QIcon icon(":icons/battery.svg");
    QPixmap pixmap(icon.pixmap(icon.actualSize(QSize(128, 128))));
    QGraphicsPixmapItem* pixmap_item = scene->addPixmap(pixmap);
    pixmap_item->setOffset(
      -pixmap.width() / 2,
      -pixmap.height() / 2);
    pixmap_item->setScale(icon_scale);
    pixmap_item->setZValue(20.0);
    pixmap_item->setPos(
      x + icon_ring_radius * cos(icon_bearing),
      y - icon_ring_radius * sin(icon_bearing));
    if (!coordinate_system.is_y_flipped())
      pixmap_item->setTransform(pixmap_item->transform().scale(1, -1));
    pixmap_item->setToolTip("This vertex is a charger");
  }

  /// For now, we only show one of the icon below as there's limited
  /// space for the icon, also "pickup_dispenser, dropoff_ingestor,
  /// is_cleaning_zone" should be exclusive to one another
  std::string icon_name;
  if (!pickup_dispenser().empty())
    icon_name = ":icons/pickup.svg";
  else if (!dropoff_ingestor().empty())
    icon_name = ":icons/dropoff.svg";
  else if (is_cleaning_zone())
    icon_name = ":icons/clean.svg";
  else if (!lift_cabin().empty())
    icon_name = ":icons/lift.svg";

  if (!icon_name.empty())
  {
    const double icon_bearing = -45.0 * M_PI / 180.0;

    QIcon icon(icon_name.c_str());
    QPixmap pixmap(icon.pixmap(icon.actualSize(QSize(128, 128))));
    QGraphicsPixmapItem* pixmap_item = scene->addPixmap(pixmap);
    pixmap_item->setOffset(
      -pixmap.width() / 2,
      -pixmap.height() / 2);
    pixmap_item->setScale(icon_scale);
    pixmap_item->setZValue(20.0);
    pixmap_item->setPos(
      x + icon_ring_radius * cos(icon_bearing),
      y - icon_ring_radius * sin(icon_bearing));
    if (!coordinate_system.is_y_flipped())
      pixmap_item->setTransform(pixmap_item->transform().scale(1, -1));
    pixmap_item->setToolTip(("Vertex is " + icon_name).c_str());
  }

  if (!name.empty())
  {
    QGraphicsSimpleTextItem* text_item = scene->addSimpleText(
      QString::fromStdString(name),
      font);
    text_item->setBrush(selected ? selected_color : vertex_color);

    QRectF bb = text_item->sceneBoundingRect();
    if (coordinate_system.is_y_flipped())
    {
      // default screen coordinates: +Y=down. Nothing special needed.
      text_item->setTransform(QTransform::fromScale(0.1, 0.1));
      text_item->setPos(
        x - 0.05 * bb.width(),
        y + 3.5 * radius - 0.05 * bb.height());
    }
    else
    {
      // if Y is not flipped, this means we have +Y=up, so we have to
      // flip the text, because Qt's default is for +Y=down screen coords
      text_item->setTransform(QTransform::fromScale(0.1, -0.1));
      text_item->setPos(
        x - 0.05 * bb.width(),
        y - 3.5 * radius + 0.05 * bb.height());
    }
  }
}

void Vertex::set_param(const std::string& param_name, const std::string& value)
{
  auto it = params.find(param_name);
  if (it == params.end())
  {
    printf("tried to set unknown parameter [%s]\n", param_name.c_str());
    return;  // unknown parameter
  }
  it->second.set(value);
}

bool Vertex::is_parking_point() const
{
  const auto it = params.find("is_parking_spot");
  if (it == params.end())
    return false;

  return it->second.value_bool;
}

bool Vertex::is_holding_point() const
{
  const auto it = params.find("is_holding_point");
  if (it == params.end())
    return false;

  return it->second.value_bool;
}

bool Vertex::is_charger() const
{
  const auto it = params.find("is_charger");
  if (it == params.end())
    return false;

  return it->second.value_bool;
}

bool Vertex::is_cleaning_zone() const
{
  const auto it = params.find("is_cleaning_zone");
  if (it == params.end())
    return false;

  return it->second.value_bool;
}

std::string Vertex::dropoff_ingestor() const
{
  const auto it = params.find("dropoff_ingestor");
  if (it == params.end())
    return "";

  return it->second.value_string;
}

std::string Vertex::pickup_dispenser() const
{
  const auto it = params.find("pickup_dispenser");
  if (it == params.end())
    return "";

  return it->second.value_string;
}

std::string Vertex::lift_cabin() const
{
  /// Note: currently lift_cabin vertex is auto-generated when adding
  /// a lift on traffic editor. Therefore lift cabin param is part of the
  /// 'allowed_params' above. For now, the param 'lift_cabin' doesn't
  /// serve any purpose in rmf building map generation and rmf graph.
  const auto it = params.find("lift_cabin");
  if (it == params.end())
    return "";

  return it->second.value_string;
}
