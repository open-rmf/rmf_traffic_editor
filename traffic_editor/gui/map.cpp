/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#include "./map.h"
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include <QFileInfo>
#include <QDir>

using std::string;
using std::cout;
using std::endl;


Map::Map()
: building_name("building"),
  changed(false)
{
}

Map::~Map()
{
}

/// Load a YAML file description of a traffic-editor map
///
/// This function replaces the contents of this object with what is
/// in the YAML file.
void Map::load_yaml(const string &filename)
{
  // This function may throw exceptions. Caller should be ready for them!
  YAML::Node y = YAML::LoadFile(filename.c_str());

  // change directory to the path of the file, so that we can correctly open
  // relative paths recorded in the file
  QString dir(QFileInfo(QString::fromStdString(filename)).absolutePath());
  qDebug("changing directory to [%s]", qUtf8Printable(dir));
  if (!QDir::setCurrent(dir))
    throw std::runtime_error("couldn't change directory");

  if (y["building_name"])
    building_name = y["building_name"].as<string>();

  if (!y["levels"] || !y["levels"].IsMap())
    throw std::runtime_error("expected top-level dictionary named 'levels'");

  levels.clear();
  const YAML::Node yl = y["levels"];
  for (YAML::const_iterator it = yl.begin(); it != yl.end(); ++it)
  {
    Level l;
    l.from_yaml(it->first.as<string>(), it->second);
    levels.push_back(l);
  }

  if (y["lifts"] && y["lifts"].IsMap())
  {
    const YAML::Node& y_lifts = y["lifts"];
    for (YAML::const_iterator it = y_lifts.begin(); it != y_lifts.end(); ++it)
    {
      Lift lift;
      lift.from_yaml(it->first.as<string>(), it->second);
      lifts.push_back(lift);
    }
  }

  changed = false;
}

bool Map::save_yaml(const std::string &filename)
{
  printf("Map::save_yaml(%s)\n", filename.c_str());

  YAML::Node y;
  y["building_name"] = building_name;

  y["levels"] = YAML::Node(YAML::NodeType::Map);
  for (const auto &level : levels)
    y["levels"][level.name] = level.to_yaml();

  y["lifts"] = YAML::Node(YAML::NodeType::Map);
  for (const auto& lift : lifts)
    y["lifts"][lift.name] = lift.to_yaml();

  YAML::Emitter emitter;
  write_yaml_node(y, emitter);
  std::ofstream fout(filename);
  fout << emitter.c_str() << std::endl;

  changed = false;
  return true;
}

void Map::add_vertex(int level_index, double x, double y)
{
  if (level_index >= static_cast<int>(levels.size()))
    return;
  levels[level_index].vertices.push_back(Vertex(x, y));
  changed = true;
}

int Map::find_nearest_vertex_index(
    int level_index, double x, double y, double &distance)
{
  double min_dist = 1e100;
  int min_index = -1;
  for (size_t i = 0; i < levels[level_index].vertices.size(); i++) {
    const Vertex &v = levels[level_index].vertices[i];
    const double dx = x - v.x;
    const double dy = y - v.y;
    const double dist2 = dx*dx + dy*dy;  // no need for sqrt each time
    if (dist2 < min_dist) {
      min_dist = dist2;
      min_index = i;
    }
  }
  distance = sqrt(min_dist);
  return min_index;  // will be -1 if vertices vector is empty
}

int Map::nearest_item_index_if_within_distance(
      const int level_index,
      const double x,
      const double y,
      const double distance_threshold,
      const ItemType item_type)
{
  if (level_index >= static_cast<int>(levels.size()))
    return -1;

  double min_dist = 1e100;
  int min_index = -1;
  if (item_type == VERTEX) {
    for (size_t i = 0; i < levels[level_index].vertices.size(); i++) {
      const Vertex &p = levels[level_index].vertices[i];
      const double dx = x - p.x;
      const double dy = y - p.y;
      const double dist2 = dx*dx + dy*dy;  // no need for sqrt each time
      if (dist2 < min_dist) {
        min_dist = dist2;
        min_index = i;
      }
    }
  }
  else if (item_type == MODEL) {
    for (size_t i = 0; i < levels[level_index].models.size(); i++) {
      const Model &m = levels[level_index].models[i];
      const double dx = x - m.x;
      const double dy = y - m.y;
      const double dist2 = dx*dx + dy*dy;  // no need for sqrt each time
      if (dist2 < min_dist) {
        min_dist = dist2;
        min_index = i;
      }
    }
  }
  if (sqrt(min_dist) < distance_threshold)
    return min_index;
  return -1;
}

void Map::add_edge(
      const int level_index,
      const int start_vertex_index,
      const int end_vertex_index,
      const Edge::Type edge_type)
{
  if (level_index >= static_cast<int>(levels.size()))
    return;

  printf("Map::add_edge(%d, %d, %d, %d)\n",
      level_index, start_vertex_index, end_vertex_index,
      static_cast<int>(edge_type));
  levels[level_index].edges.push_back(
      Edge(start_vertex_index, end_vertex_index, edge_type));
  changed = true;
}

bool Map::delete_selected(const int level_index)
{
  if (level_index >= static_cast<int>(levels.size()))
    return false;

  printf("Map::delete_keypress()\n");
  if (!levels[level_index].delete_selected())
    return false;

  changed = true;
  return true;
}

void Map::add_model(
    const int level_idx,
    const double x,
    const double y,
    const double yaw,
    const std::string &model_name)
{
  if (level_idx >= static_cast<int>(levels.size()))
    return;

  printf("Map::add_model(%d, %.1f, %.1f, %.2f, %s)\n",
      level_idx, x, y, yaw, model_name.c_str());
  levels[level_idx].models.push_back(
      Model(x, y, yaw, model_name, model_name));
  changed = true;
}

void Map::set_model_yaw(
    const int level_idx,
    const int model_idx,
    const double yaw)
{
  if (level_idx >= static_cast<int>(levels.size()))
    return;

  levels[level_idx].models[model_idx].yaw = yaw;
  changed = true;
}

void Map::remove_polygon_vertex(
    const int level_idx,
    const int polygon_idx,
    const int vertex_idx)
{
  if (level_idx < 0 || level_idx > static_cast<int>(levels.size()))
    return;  // oh no
  levels[level_idx].remove_polygon_vertex(polygon_idx, vertex_idx);
  changed = true;
}

int Map::polygon_edge_drag_press(
    const int level_idx,
    const int polygon_idx,
    const double x,
    const double y)
{
  if (level_idx < 0 || level_idx > static_cast<int>(levels.size()))
    return -1;  // oh no
  return levels[level_idx].polygon_edge_drag_press(polygon_idx, x, y);
}

void Map::clear()
{
  changed = true;
  building_name = "";
  levels.clear();
}

void Map::add_level(const Level &new_level)
{
  // make sure we don't have this level already
  for (const auto &level : levels)
    if (level.name == new_level.name)
      return;
  changed = true;
  levels.push_back(new_level);
}

// Recursive function to write YAML ordered maps. Credit: Dave Hershberger
// posted to this GitHub issue: https://github.com/jbeder/yaml-cpp/issues/169
void Map::write_yaml_node(const YAML::Node& node, YAML::Emitter& emitter)
{
  switch (node.Style())
  {
    case YAML::EmitterStyle::Block:
      emitter << YAML::Block;
      break;
    case YAML::EmitterStyle::Flow:
      emitter << YAML::Flow;
      break;
    default:
      break;
  }

  switch (node.Type())
  {
    case YAML::NodeType::Sequence:
    {
      emitter << YAML::BeginSeq;
      for (size_t i = 0; i < node.size(); i++)
        write_yaml_node(node[i], emitter);
      emitter << YAML::EndSeq;
      break;
    }
    case YAML::NodeType::Map:
    {
      emitter << YAML::BeginMap;
      // the keys are stored in random order, so we need to collect and sort
      std::vector<string> keys;
      keys.reserve(node.size());
      for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
        keys.push_back(it->first.as<string>());
      std::sort(keys.begin(), keys.end());
      for (size_t i = 0; i < keys.size(); i++)
      {
        emitter << YAML::Key << keys[i] << YAML::Value;
        write_yaml_node(node[keys[i]], emitter);
      }
      emitter << YAML::EndMap;
      break;
    }
    default:
      emitter << node;
      break;
  }
}
