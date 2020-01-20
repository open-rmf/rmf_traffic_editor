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

/// The level_name parameter is required in order to know how to draw the
/// doors, since many lifts have more than one set of doors, which open on
/// some but not all floors. It's not being used (yet).
void Lift::draw(
    QGraphicsScene *scene,
    const double meters_per_pixel,
    const string& /*level_name*/,
    const bool apply_transformation) const
{
  const double cabin_w = width / meters_per_pixel;
  const double cabin_d = depth / meters_per_pixel;
  QPen cabin_pen(Qt::black);
  cabin_pen.setWidth(0.05 / meters_per_pixel);

  QGraphicsRectItem *cabin_rect = new QGraphicsRectItem(
      -cabin_w / 2.0,
      -cabin_d / 2.0,
      cabin_w,
      cabin_d);
  cabin_rect->setPen(cabin_pen);
  cabin_rect->setBrush(QBrush(QColor::fromRgbF(1.0, 1.0, 0.0, 0.5)));
  scene->addItem(cabin_rect);

  QList<QGraphicsItem *> items;
  items.append(cabin_rect);

  if (!name.empty()) {
    QFont font("Helvetica");
    font.setPointSize(0.2 / meters_per_pixel);
    QGraphicsSimpleTextItem *text_item = scene->addSimpleText(
        QString::fromStdString(name), font);
    text_item->setBrush(QColor(255, 0, 0, 255));
    text_item->setPos(-cabin_w / 3.0, 0.0);

    // todo: set font size to something reasonable
    // todo: center-align text?
    items.append(text_item);
  }

  for (const LiftDoor& door : doors)
  {
    printf("rendering door %s\n", door.name.c_str());
    const double door_x = door.x / meters_per_pixel;
    const double door_y = -door.y / meters_per_pixel;
    const double door_w = door.width / meters_per_pixel;
    const double door_thickness = 0.2 / meters_per_pixel;
    QGraphicsRectItem *door_item = new QGraphicsRectItem(
        door_x - door_w / 2.0,
        door_y - door_thickness / 2.0,
        door_w,
        door_thickness);
    door_item->setRotation(-180.0 / 3.1415926 * door.motion_axis_orientation);

    QPen door_pen(Qt::red);
    door_pen.setWidth(0.05 / meters_per_pixel);
    door_item->setPen(door_pen);
    door_item->setBrush(QBrush(QColor::fromRgbF(1.0, 0.0, 0.0, 0.5)));

    items.append(door_item);
  }

  QGraphicsItemGroup *group = scene->createItemGroup(items);

  if (apply_transformation)
  {
    group->setRotation(-180.0 / 3.1415926 * yaw);
    group->setPos(x, y);
  }
}
