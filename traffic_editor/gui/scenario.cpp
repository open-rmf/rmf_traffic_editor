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

#include <fstream>

#include "scenario.h"
#include "yaml_utils.h"

using std::string;


Scenario::Scenario()
{
}

Scenario::~Scenario()
{
}

bool Scenario::load()
{
  YAML::Node yaml;
  try
  {
    yaml = YAML::LoadFile(filename.c_str());
  }
  catch (const std::exception& e)
  {
    printf("couldn't parse %s: %s", filename.c_str(), e.what());
    return false;
  }

  if (yaml["name"])
    name = yaml["name"].as<string>();

  levels.clear();
  const YAML::Node yl = yaml["levels"];
  for (YAML::const_iterator it = yl.begin(); it != yl.end(); ++it)
  {
    ScenarioLevel l;
    l.from_yaml(it->first.as<string>(), it->second);
    levels.push_back(l);
  }

  return true;
}

bool Scenario::save() const
{
  YAML::Node y;
  y["version"] = 1;
  y["name"] = name;

  y["levels"] = YAML::Node(YAML::NodeType::Map);
  for (const ScenarioLevel& level : levels)
    y["levels"][level.name] = level.to_yaml();

  YAML::Emitter emitter;
  yaml_utils::write_node(y, emitter);
  std::ofstream fout(filename);
  fout << emitter.c_str() << std::endl;

  return true;
}

void Scenario::draw(
    QGraphicsScene *scene,
    const std::string& level_name,
    const double meters_per_pixel) const
{
  printf("Scenario::draw(%s)\n", level_name.c_str());
  for (const ScenarioLevel& level : levels)
    if (level.name == level_name)
    {
      level.draw(scene, meters_per_pixel);
      break;
    }
}

void Scenario::add_vertex(
    const std::string& level_name,
    const double x,
    const double y)
{
  printf("Scenario::add_vertex(%s, %.1f, %.1f)\n", level_name.c_str(), x, y);
  for (ScenarioLevel& level : levels)
    if (level.name == level_name)
    {
      level.add_vertex(x, y);
      return;
    }
  // if we get here, we didn't find a ScenarioLevel for this level name,
  // so we have to add it now.
  printf("adding level [%s] to scenario\n", level_name.c_str());
  ScenarioLevel level;
  level.name = level_name;
  level.add_vertex(x, y);
  levels.push_back(level);
}

void Scenario::clear_selection(const std::string& level_name)
{
  for (ScenarioLevel& level : levels)
    if (level.name == level_name)
      level.clear_selection();
}

bool Scenario::delete_selected(const std::string& level_name)
{
  for (ScenarioLevel& level : levels)
    if (level.name == level_name)
      return level.delete_selected();
  return true;
}
