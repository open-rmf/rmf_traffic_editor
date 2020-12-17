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

#include <cmath>

#include <QGraphicsScene>
#include <QGraphicsSimpleTextItem>

#include "traffic_editor/fiducial.h"
using std::string;

Fiducial::Fiducial()
{
  uuid = QUuid::createUuid();
}

Fiducial::Fiducial(double _x, double _y, const string& _name)
: x(_x), y(_y), name(_name)
{
}

void Fiducial::from_yaml(const YAML::Node& data)
{
  if (!data.IsSequence())
    throw std::runtime_error("Vertex::from_yaml expected a sequence");
  x = data[0].as<double>();
  y = data[1].as<double>();
  name = data[2].as<string>();
}

YAML::Node Fiducial::to_yaml() const
{
  // This is in image space. I think it's safe to say nobody is clicking
  // with more than 1/1000 precision inside a single pixel.
  YAML::Node node;
  node.SetStyle(YAML::EmitterStyle::Flow);
  node.push_back(std::round(x * 1000.0) / 1000.0);
  node.push_back(std::round(y * 1000.0) / 1000.0);
  node.push_back(name);
  return node;
}

void Fiducial::draw(
  QGraphicsScene* scene,
  const double meters_per_pixel) const
{
  const double a = 0.5;
  const QColor color = QColor::fromRgbF(0.0, 0.0, 1.0, a);
  const QColor selected_color = QColor::fromRgbF(1.0, 0.0, 0.0, a);

  QPen pen(selected ? selected_color : color);
  pen.setWidth(0.2 / meters_per_pixel);
  const double radius = 0.5 / meters_per_pixel;

  scene->addEllipse(
    x - radius,
    y - radius,
    2 * radius,
    2 * radius,
    pen);
  scene->addLine(x, y - 2 * radius, x, y + 2 * radius, pen);
  scene->addLine(x - 2 * radius, y, x + 2 * radius, y, pen);

  if (!name.empty())
  {
    QGraphicsSimpleTextItem* item = scene->addSimpleText(
      QString::fromStdString(name));
    item->setBrush(QColor(0, 0, 255, 255));
    item->setPos(x, y + radius);
  }
}

double Fiducial::distance(const Fiducial& f)
{
  const double dx = f.x - x;
  const double dy = f.y - y;
  return std::sqrt(dx*dx + dy*dy);
}
