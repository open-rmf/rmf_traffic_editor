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
  _uuid = QUuid::createUuid();
}

Feature::Feature(double x, double y)
: _x(x), _y(y)
{
  _uuid = QUuid::createUuid();
}

void Feature::from_yaml(const YAML::Node& data)
{
  if (!data.IsMap())
    throw std::runtime_error("Feature::from_yaml() expected a map");
  _x = data["x"].as<double>();
  _y = data["y"].as<double>();
  _uuid = QString(data["uuid"].as<std::string>().c_str());

  // todo: parse name, if it's in the YAML
}

YAML::Node Feature::to_yaml() const
{
  YAML::Node node;
  node.SetStyle(YAML::EmitterStyle::Flow);
  node["x"] = std::round(_x * 1000.0) / 1000.0;
  node["y"] = std::round(_y * 1000.0) / 1000.0;
  node["name"] = _name;
  node["uuid"] = _uuid.toString().toStdString();
  return node;
}

void Feature::draw(
  QGraphicsScene* scene,
  const double meters_per_pixel,
  const QColor color) const
{
  // todo: compute selected color based on color param
  const QColor selected_color = QColor::fromRgbF(1.0, 0.0, 0.0, 0.5);

  QPen pen(_selected ? selected_color : color);
  pen.setWidth(0.05 / meters_per_pixel);
  const double radius = 0.10 / meters_per_pixel;

  QGraphicsEllipseItem* circle = scene->addEllipse(
    _x - radius,
    _y - radius,
    2 * radius,
    2 * radius,
    pen,
    QBrush(QColor::fromRgbF(1, 1, 1, 0.5)));
  circle->setZValue(200.0);

  QGraphicsLineItem* vertical_line = scene->addLine(
    _x,
    _y - radius,
    _x,
    _y + radius,
    pen);
  vertical_line->setZValue(200.0);

  QGraphicsLineItem* horizontal_line = scene->addLine(
    _x - radius,
    _y,
    _x + radius,
    _y,
    pen);
  horizontal_line->setZValue(200.0);

  /*
  QGraphicsSimpleTextItem* item = scene->addSimpleText(QString::number(_id));
  item->setBrush(QColor(0, 0, 255, 255));
  item->setPos(_x, _y + radius);
  auto font = item->font();
  font.setPointSize(30);
  item->setFont(font);
  */
}
