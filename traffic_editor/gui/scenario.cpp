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

#ifdef HAS_IGNITION_PLUGIN
#include <ignition/common/SystemPaths.hh>
#endif

#include "scenario.h"
#include "yaml_utils.h"

using std::string;
using std::unique_ptr;


Scenario::Scenario()
{
}

Scenario::~Scenario()
{
}

bool Scenario::load()
{
  printf("Scenario::load(%s)\n", filename.c_str());
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
  if (yaml["levels"])
  {
    const YAML::Node yl = yaml["levels"];
    for (YAML::const_iterator it = yl.begin(); it != yl.end(); ++it)
    {
      ScenarioLevel l;
      l.from_yaml(it->first.as<string>(), it->second);
      levels.push_back(l);
    }
  }

#ifdef HAS_IGNITION_PLUGIN
  if (yaml["plugin_name"])
  {
    string plugin_path = yaml["plugin_path"].as<string>();
    ignition::common::SystemPaths paths;
    paths.AddPluginPaths(plugin_path);

    const string plugin_name = yaml["plugin_name"].as<string>();
    std::string lib_path = paths.FindSharedLibrary(plugin_name);
    printf("FindSharedLibrary returned [%s]\n", lib_path.c_str());

    ignition::plugin::Loader loader;
    std::unordered_set<string> plugin_libs = loader.LoadLib(lib_path);
    std::unordered_set<string> sim_libs =
      loader.PluginsImplementing("Simulation");

    for (const auto& s : plugin_libs)
      printf("  found plugin library: [%s]\n", s.c_str());

    for (const auto& s : sim_libs)
      printf("  found simulation library: [%s]\n", s.c_str());

    for (const std::string& plugin_class_name : plugin_libs)
    {
      if (sim_libs.find(plugin_class_name) != sim_libs.end())
      {
        printf(
          "trying to instantiate [%s] from library [%s]...\n",
          plugin_class_name.c_str(),
          plugin_name.c_str());
        sim_plugin = loader.Instantiate(plugin_class_name);

        if (sim_plugin.IsEmpty())
        {
          printf("simulation plugin instantiation failed :(\n");
          break;
        }

        printf("success! created a simulation plugin instance!\n");
        Simulation* sim = sim_plugin->QueryInterface<Simulation>();
        if (!sim)
        {
          printf("woah! couldn't get interface to plugin!\n");
          break;
        }

        sim->load(yaml["plugin_config"]);
        break;
      }
    }
  }
#endif

  print();

  return true;
}

void Scenario::print() const
{
  printf("scenario: [%s]\n", name.c_str());
  printf("  filename: [%s]\n", filename.c_str());
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
  QGraphicsScene* scene,
  const std::string& level_name,
  const double meters_per_pixel,
  std::vector<EditorModel>& /*editor_models*/) const
{
  printf("Scenario::draw(%s)\n", level_name.c_str());
  for (const ScenarioLevel& level : levels)
  {
    if (level.name == level_name)
    {
      level.draw(scene, meters_per_pixel);
      break;
    }
  }
}

void Scenario::add_vertex(
  const std::string& level_name,
  const double x,
  const double y)
{
  printf("Scenario::add_vertex(%s, %.1f, %.1f)\n", level_name.c_str(), x, y);
  for (ScenarioLevel& level : levels)
  {
    if (level.name == level_name)
    {
      level.add_vertex(x, y);
      return;
    }
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
  {
    if (level.name == level_name)
      level.clear_selection();
  }
}

bool Scenario::delete_selected(const std::string& level_name)
{
  for (ScenarioLevel& level : levels)
  {
    if (level.name == level_name)
      return level.delete_selected();
  }
  return true;
}

#ifdef HAS_IGNITION_PLUGIN
void Scenario::sim_tick(Building& building)
{
  if (!sim_plugin.IsEmpty())
  {
    std::lock_guard<std::mutex> building_guard(building.building_mutex);
    Simulation* sim = sim_plugin->QueryInterface<Simulation>();
    if (sim)
      sim->tick(building);
  }
}

void Scenario::sim_reset(Building& building)
{
  if (!sim_plugin.IsEmpty())
  {
    Simulation* sim = sim_plugin->QueryInterface<Simulation>();
    if (sim)
      sim->reset(building);
  }

  sim_time_seconds = 0.0;
  sim_tick_counter = 0;
}

void Scenario::clear_scene()
{
  printf("Scenario::clear_scene()\n");

  if (!sim_plugin.IsEmpty())
  {
    Simulation* sim = sim_plugin->QueryInterface<Simulation>();
    if (sim)
      sim->scene_clear();
  }
}

void Scenario::scene_update(
  QGraphicsScene* scene,
  Building& building,
  const int level_idx)
{
  if (!sim_plugin.IsEmpty())
  {
    std::lock_guard<std::mutex> building_guard(building.building_mutex);
    Simulation* sim = sim_plugin->QueryInterface<Simulation>();
    if (sim)
      sim->scene_update(scene, building, level_idx);
  }
}
#endif
