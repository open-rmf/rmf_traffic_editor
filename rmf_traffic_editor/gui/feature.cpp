/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#include <QString>

#include "feature.hpp"


Feature::Feature()
{
  _id = QUuid::createUuid();
}

Feature::Feature(double x, double y)
: _x(x), _y(y)
{
  _id = QUuid::createUuid();
}

void Feature::from_yaml(const YAML::Node& data)
{
  if (!data.IsMap())
    throw std::runtime_error("Feature::from_yaml() expected a map");
  _x = data["x"].as<double>();
  _y = data["y"].as<double>();
  _id = QString(data["id"].as<std::string>().c_str());

  // todo: parse name, if it's in the YAML
}

YAML::Node Feature::to_yaml() const
{
  YAML::Node node;
  node.SetStyle(YAML::EmitterStyle::Flow);
  node["x"] = std::round(_x * 1000.0) / 1000.0;
  node["y"] = std::round(_y * 1000.0) / 1000.0;
  node["name"] = _name;
  node["id"] = _id.toString().toStdString();
  return node;
}

void Feature::draw(
  QGraphicsScene* scene,
  const double meters_per_pixel,
  const QColor color) const
{
  const QColor selected_color = QColor::fromRgbF(1.0, 0.0, 0.0, 0.5);

  const double pen_width = 0.025 / meters_per_pixel;
  QPen pen(
    QBrush(_selected ? selected_color : color),
    pen_width,
    Qt::SolidLine,
    Qt::FlatCap);

  const double radius = radius_meters / meters_per_pixel;

  QGraphicsEllipseItem* circle = scene->addEllipse(
    _x - radius,
    _y - radius,
    2 * radius,
    2 * radius,
    pen,
    QBrush(QColor::fromRgbF(1, 1, 1, 0.5)));
  circle->setZValue(200.0);

  const double line_radius = (radius - pen_width / 2.0) / sqrt(2.0);
  QGraphicsLineItem* line_1 = scene->addLine(
    _x - line_radius,
    _y - line_radius,
    _x + line_radius,
    _y + line_radius,
    pen);
  line_1->setZValue(200.0);

  QGraphicsLineItem* line_2 = scene->addLine(
    _x - line_radius,
    _y + line_radius,
    _x + line_radius,
    _y - line_radius,
    pen);
  line_2->setZValue(200.0);

  /*
  QGraphicsSimpleTextItem* item = scene->addSimpleText(QString::number(_id));
  item->setBrush(QColor(0, 0, 255, 255));
  item->setPos(_x, _y + radius);
  auto font = item->font();
  font.setPointSize(30);
  item->setFont(font);
  */
}
