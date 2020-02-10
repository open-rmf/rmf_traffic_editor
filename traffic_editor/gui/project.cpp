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

#include "project.h"
#include "yaml_utils.h"

#include <QFileInfo>
#include <QDir>

using std::string;


Project::Project()
{
}

Project::~Project()
{
}

bool Project::load_yaml_file(const std::string& _filename)
{
  filename = _filename;

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

  // change directory to the path of the file, so that we can correctly open
  // relative paths recorded in the file

  // TODO: save previous directory and restore it, in case other files
  // we load are in different directories (!)

  QString dir(QFileInfo(QString::fromStdString(filename)).absolutePath());
  printf("changing directory to [%s]", qUtf8Printable(dir));
  if (!QDir::setCurrent(dir))
  {
    printf("couldn't change directory\n");
    return false;
  }

  if (yaml["name"])
    name = yaml["name"].as<string>();

  if (yaml["building"] && yaml["building"].IsMap())
  {
    if (!yaml["building"]["filename"])
    {
      printf("expected a 'filename' key within the 'building' map\n");
      return false;
    }
    building.filename = yaml["building"]["filename"].as<string>();
    if (!building.load_yaml_file())
      return false;
  }

  if (yaml["scenarios"] && yaml["scenarios"].IsSequence())
  {
    for (YAML::const_iterator scenario_node = yaml["scenarios"].begin();
        scenario_node != yaml["scenarios"].end();
        ++scenario_node)
    {
      Scenario scenario;
      scenario.filename = (*scenario_node)["filename"].as<string>();
      if (!scenario.load())
      {
        printf("couldn't load [%s]\n", scenario.filename.c_str());
        //return false;
      }
      scenarios.push_back(scenario);
    }
    if (!scenarios.empty())
      scenario_idx = 0;
  }

  return true;
}

bool Project::save_yaml_file() const
{
  printf("Project::save_yaml_file()\n");

  YAML::Node y;
  y["version"] = 1;
  y["name"] = name;

  y["building"] = YAML::Node(YAML::NodeType::Map);
  y["building"]["filename"] = building.filename;

  for (const auto& scenario : scenarios)
  {
    printf("saving scenario\n");
    YAML::Node scenario_node;
    scenario_node["filename"] = scenario.filename;
    y["scenarios"].push_back(scenario_node);
  }

  YAML::Emitter emitter;
  yaml_utils::write_node(y, emitter);
  std::ofstream fout(filename);
  fout << emitter.c_str() << std::endl;

  return true;
}

bool Project::save()
{
  if (!save_yaml_file())
    return false;

  building.save_yaml_file();

  for (const auto& scenario : scenarios)
    if (!scenario.save())
      return false;

  return true;
}

bool Project::load(const std::string& _filename)
{
  // future extension point: dispatch based on file type (json/yaml/...)
  return load_yaml_file(_filename);
}

void Project::add_scenario_vertex(
    const int level_idx,
    const double x,
    const double y)
{
  printf("add_scenario_vertex(%d, %.3f, %.3f)\n", level_idx, x, y);
  if (scenario_idx < 0 || scenario_idx >= static_cast<int>(scenarios.size()))
    return;
  scenarios[scenario_idx].add_vertex(building.levels[level_idx].name, x, y);
}

void Project::scenario_row_clicked(const int row)
{
  printf("Project::scenario_row_clicked(%d)\n", row);
  if (row < 0 || row >= static_cast<int>(scenarios.size()))
  {
    scenario_idx = -1;
    return;
  }
  scenario_idx = row;
}

void Project::draw(
    QGraphicsScene *scene,
    const int level_idx,
    std::vector<EditorModel>& editor_models)
{
  if (building.levels.empty())
  {
    printf("nothing to draw!\n");
    return;
  }

  building.levels[level_idx].draw(scene, editor_models);
  building.draw_lifts(scene, level_idx);
  
  if (scenario_idx >= 0)
    scenarios[scenario_idx].draw(
        scene,
        building.levels[level_idx].name,
        building.levels[level_idx].drawing_meters_per_pixel);
}
