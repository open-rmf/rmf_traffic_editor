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

#include "traffic_editor/correspondence_point.hpp"


uint16_t CorrespondencePoint::next_id_ = 0;

CorrespondencePoint::CorrespondencePoint()
{
  uuid_ = QUuid::createUuid();
}

CorrespondencePoint::CorrespondencePoint(double x, double y)
: x_(x), y_(y)
{
  id_ = next_id_++;
  uuid_ = QUuid::createUuid();
}

void CorrespondencePoint::from_yaml(const YAML::Node& data)
{
  if (!data.IsMap()) {
    throw std::runtime_error("CorrespondencePoint::from_yaml() expected a map");
  }
  x_ = data["x"].as<double>();
  y_ = data["y"].as<double>();
  id_ = data["id"].as<uint16_t>();
  uuid_ = QString(data["uuid"].as<std::string>().c_str());
}

YAML::Node CorrespondencePoint::to_yaml() const
{
  YAML::Node node;
  node.SetStyle(YAML::EmitterStyle::Flow);
  node["x"] = std::round(x_ * 1000.0) / 1000.0;
  node["y"] = std::round(y_ * 1000.0) / 1000.0;
  node["id"] = id_;
  node["uuid"] = uuid_.toString().toStdString();
  return node;
}

void CorrespondencePoint::draw(QGraphicsScene* scene, double meters_per_pixel) const
{
  const double a = 0.5;
  const QColor color = QColor::fromRgbF(0.0, 0.0, 1.0, a);
  const QColor selected_color = QColor::fromRgbF(1.0, 0.0, 0.0, a);

  QPen pen(selected_ ? selected_color : color);
  pen.setWidth(0.2 / meters_per_pixel);
  const double radius = 0.5 / meters_per_pixel;

  scene->addLine(x_ - 2 * radius, y_ - 2 * radius, x_ + 2 * radius, y_ + 2 * radius, pen);
  scene->addLine(x_ - 2 * radius, y_ + 2 * radius, x_ + 2 * radius, y_ - 2 * radius, pen);

  QGraphicsSimpleTextItem* item = scene->addSimpleText(
    QString::number(id_));
  item->setBrush(QColor(0, 0, 255, 255));
  item->setPos(x_ + radius, y_ + radius);
}
