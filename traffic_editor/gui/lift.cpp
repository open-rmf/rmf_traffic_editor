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

#include <QGraphicsScene>
#include <QGraphicsSimpleTextItem>


#include "lift.h"
using std::string;


Lift::Lift()
{
}

void Lift::from_yaml(const std::string& _name, const YAML::Node &data)
{
  if (!data.IsMap())
    throw std::runtime_error("Lift::from_yaml() expected a map");
  x = data["x"].as<double>();
  y = data["y"].as<double>();
  yaw = data["yaw"].as<double>();
  name = _name;
  reference_floor_name = data["reference_floor_name"].as<string>();
  width = data["width"].as<double>();
  depth = data["depth"].as<double>();

  if (data["doors"] && data["doors"].IsMap())
  {
    const YAML::Node& yd = data["doors"];
    for (YAML::const_iterator it = yd.begin(); it != yd.end(); ++it)
    {
      LiftDoor door;
      door.from_yaml(it->first.as<string>(), it->second);
      doors.push_back(door);
    }
  }

  if (data["level_door"] && data["level_door"].IsMap())
  {
    const YAML::Node ym = data["level_door"];
    for (YAML::const_iterator it = ym.begin(); it != ym.end(); ++it)
      level_door[it->first.as<string>()] = it->second.as<string>();
  }
}

YAML::Node Lift::to_yaml() const
{
  // This is in image space. I think it's safe to say nobody is clicking
  // with more than 1/1000 precision inside a single pixel.

  YAML::Node n;
  n["x"] = round(x * 1000.0) / 1000.0;
  n["y"] = round(y * 1000.0) / 1000.0;
  // let's give yaw another decimal place because, I don't know, reasons (?)
  n["yaw"] = round(yaw * 10000.0) / 10000.0;
  n["reference_floor_name"] = reference_floor_name;
  n["width"] = round(width * 1000.0) / 1000.0;
  n["depth"] = round(depth * 1000.0) / 1000.0;

  n["doors"] = YAML::Node(YAML::NodeType::Map);
  for (const auto& door : doors)
    n["doors"][door.name] = door.to_yaml();

  n["level_door"] = YAML::Node(YAML::NodeType::Map);
  std::map<string, string>::const_iterator it;
  for (it = level_door.begin(); it != level_door.end(); ++it)
    n["level_door"][it->first] = it->second;
  return n;
}

void Lift::draw(
    QGraphicsScene *scene,
    const double meters_per_pixel,
    const string& level_name) const
{
  // todo: something more fancy than this...

  QPen pen(Qt::black);
  pen.setWidth(0.05 / meters_per_pixel);
  const double w = width / meters_per_pixel;
  const double d = depth / meters_per_pixel;
  const QColor color = QColor::fromRgbF(1.0, 1.0, 0.0, 0.5);

  scene->addRect(
      x - w,
      y - d,
      2 * w,
      2 * d,
      pen,
      QBrush(color));

  if (!name.empty()) {
    QGraphicsSimpleTextItem *item = scene->addSimpleText(
        QString::fromStdString(name));
    item->setBrush(QColor(255, 0, 0, 255));
    item->setPos(x + w, y + d);
  }
}
