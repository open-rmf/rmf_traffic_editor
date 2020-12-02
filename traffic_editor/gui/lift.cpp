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
#include <QGraphicsSimpleTextItem>

#include "traffic_editor/lift.h"
using std::string;


Lift::Lift()
{
}

void Lift::from_yaml(const std::string& _name, const YAML::Node& data,
  const std::vector<BuildingLevel>& levels)
{
  if (!data.IsMap())
    throw std::runtime_error("Lift::from_yaml() expected a map");
  x = data["x"].as<double>();
  y = data["y"].as<double>();
  yaw = data["yaw"].as<double>();
  name = _name;
  reference_floor_name = data["reference_floor_name"].as<string>();
  if (data["initial_floor_name"])
    initial_floor_name = data["initial_floor_name"].as<string>();
  else
    initial_floor_name = reference_floor_name;
  width = data["width"].as<double>();
  depth = data["depth"].as<double>();
  if (data["plugins"])
    plugins = data["plugins"].as<bool>();
  else
    plugins = true;

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

  if (data["highest_floor"])
    highest_floor = data["highest_floor"].as<string>();
  if (data["lowest_floor"])
    lowest_floor = data["lowest_floor"].as<string>();
  for (const auto& level : levels)
  {
    if (level.name == highest_floor)
      highest_elevation = level.elevation;
    if (level.name == lowest_floor)
      lowest_elevation = level.elevation;
  }

  // for every level, load if every door can open
  if (data["level_doors"] && data["level_doors"].IsMap())
  {
    const YAML::Node ym = data["level_doors"];
    for (YAML::const_iterator it = ym.begin(); it != ym.end(); ++it)
    {
      const std::string level_name = it->first.as<string>();
      const YAML::Node& ds = it->second;  // doors sequence node
      if (ds.IsSequence())
      {
        for (YAML::const_iterator dit = ds.begin(); dit != ds.end(); ++dit)
        {
          const std::string door_name = (*dit).as<string>();
          level_doors[level_name].push_back(door_name);
        }
      }
    }
  }
}

YAML::Node Lift::to_yaml() const
{
  // This is in image space. I think it's safe to say nobody is clicking
  // with more than 1/1000 precision inside a single pixel.

  YAML::Node n;
  n["x"] = std::round(x * 1000.0) / 1000.0;
  n["y"] = std::round(y * 1000.0) / 1000.0;
  // let's give yaw another decimal place because, I don't know, reasons (?)
  n["yaw"] = std::round(yaw * 10000.0) / 10000.0;
  n["reference_floor_name"] = reference_floor_name;
  n["highest_floor"] = highest_floor;
  n["lowest_floor"] = lowest_floor;
  n["initial_floor_name"] = initial_floor_name;
  n["width"] = std::round(width * 1000.0) / 1000.0;
  n["depth"] = std::round(depth * 1000.0) / 1000.0;
  n["plugins"] = plugins;

  n["doors"] = YAML::Node(YAML::NodeType::Map);
  for (const auto& door : doors)
    n["doors"][door.name] = door.to_yaml();

  n["level_doors"] = YAML::Node(YAML::NodeType::Map);
  for (LevelDoorMap::const_iterator level_it = level_doors.begin();
    level_it != level_doors.end();
    ++level_it)
  {
    const DoorNameList& dlist = level_it->second;
    for (DoorNameList::const_iterator door_it = dlist.begin();
      door_it != dlist.end();
      ++door_it)
    {
      n["level_doors"][level_it->first].push_back(*door_it);
      n["level_doors"][level_it->first].SetStyle(YAML::EmitterStyle::Flow);
    }
  }
  return n;
}

/// The level_name parameter is required in order to know how to draw the
/// doors, since many lifts have more than one set of doors, which open on
/// some but not all floors. It's not being used (yet).
void Lift::draw(
  QGraphicsScene* scene,
  const double meters_per_pixel,
  const string& level_name,
  const double elevation,
  const bool apply_transformation,
  const double scale,
  const double translate_x,
  const double translate_y) const
{
  if (elevation > highest_elevation || elevation < lowest_elevation)
    return;
  const double cabin_w = width / meters_per_pixel;
  const double cabin_d = depth / meters_per_pixel;
  QPen cabin_pen(Qt::black);
  cabin_pen.setWidth(0.05 / meters_per_pixel);

  QGraphicsRectItem* cabin_rect = new QGraphicsRectItem(
    -cabin_w / 2.0,
    -cabin_d / 2.0,
    cabin_w,
    cabin_d);
  cabin_rect->setPen(cabin_pen);
  auto it = level_doors.find(level_name);
  if (it == level_doors.end())
    cabin_rect->setBrush(QBrush(QColor::fromRgbF(1.0, 0.3, 0.3, 0.3)));
  else
    cabin_rect->setBrush(QBrush(QColor::fromRgbF(0.5, 1.0, 0.5, 0.5)));
  scene->addItem(cabin_rect);

  QList<QGraphicsItem*> items;
  items.append(cabin_rect);

  if (!name.empty())
  {
    QFont font("Helvetica");
    font.setPointSize(0.2 / meters_per_pixel);
    QGraphicsSimpleTextItem* text_item = scene->addSimpleText(
      QString::fromStdString(name), font);
    text_item->setBrush(QColor(255, 0, 0, 255));
    text_item->setPos(-cabin_w / 3.0, 0.0);

    // todo: set font size to something reasonable
    // todo: center-align text?
    items.append(text_item);
  }

  if (it != level_doors.end())
  {
    for (const LiftDoor& door : doors)
    {
      if (find(it->second.begin(), it->second.end(), door.name)
        == it->second.end())
        continue;
      const double door_x = door.x / meters_per_pixel;
      const double door_y = -door.y / meters_per_pixel;
      const double door_w = door.width / meters_per_pixel;
      const double door_thickness = 0.2 / meters_per_pixel;
      QGraphicsRectItem* door_item = new QGraphicsRectItem(
        -door_w / 2.0,
        -door_thickness / 2.0,
        door_w,
        door_thickness);
      door_item->setRotation(-180.0 / 3.1415926 * door.motion_axis_orientation);
      door_item->setPos(door_x, door_y);

      QPen door_pen(Qt::red);
      door_pen.setWidth(0.05 / meters_per_pixel);
      door_item->setPen(door_pen);
      door_item->setBrush(QBrush(QColor::fromRgbF(1.0, 0.0, 0.0, 0.5)));

      items.append(door_item);
    }
  }

  QGraphicsItemGroup* group = scene->createItemGroup(items);

  if (apply_transformation)
  {
    group->setRotation(-180.0 / 3.1415926 * yaw);
    group->setPos(x * scale + translate_x, y * scale + translate_y);
  }
}

bool Lift::level_door_opens(
  const std::string& level_name,
  const std::string& door_name,
  const std::vector<BuildingLevel>& levels) const
{
  LevelDoorMap::const_iterator level_it = level_doors.find(level_name);
  if (level_it == level_doors.end())
    return false;
  for (const auto& level : levels)
  {
    if (level.name == level_name)
    {
      if (level.elevation < lowest_elevation ||
        level.elevation > highest_elevation)
        return false;
      break;
    }
  }
  const DoorNameList& names = level_it->second;
  if (std::find(names.begin(), names.end(), door_name) == names.end())
    return false;
  return true;
}
