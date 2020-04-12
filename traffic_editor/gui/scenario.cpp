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

  behaviors.clear();
  if (yaml["behaviors"])
  {
    const YAML::Node yb = yaml["behaviors"];
    for (YAML::const_iterator it = yb.begin(); it != yb.end(); ++it)
      behaviors.push_back(
          unique_ptr<Behavior>(
              new Behavior(it->first.as<string>(), it->second)));
  }

  behavior_schedule.clear();
  if (yaml["behavior_schedule"])
  {
    const YAML::Node yb = yaml["behavior_schedule"];
    for (YAML::const_iterator it = yb.begin(); it != yb.end(); ++it)
    {
      BehaviorScheduleItem bsi;
      if (bsi.from_yaml(*it))
        behavior_schedule.push_back(bsi);
    }
  }

  print();

  return true;
}

void Scenario::print() const
{
  printf("scenario: [%s]\n", name.c_str());
  printf("  filename: [%s]\n", filename.c_str());
  printf("  behaviors:\n");
  for (const auto& behavior : behaviors)
    behavior->print();
  printf("  schedule:\n");
  for (const auto& schedule_item : behavior_schedule)
    schedule_item.print();
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
    const double meters_per_pixel,
    std::vector<EditorModel>& editor_models) const
{
  printf("Scenario::draw(%s)\n", level_name.c_str());
  for (const ScenarioLevel& level : levels)
    if (level.name == level_name)
    {
      level.draw(scene, meters_per_pixel);
      break;
    }

  draw_models(scene, level_name, meters_per_pixel, editor_models);
}

void Scenario::draw_models(
    QGraphicsScene *scene,
    const std::string& level_name,
    const double meters_per_pixel,
    std::vector<EditorModel>& editor_models) const
{
  for (const auto& model : models)
    if (model->state.level_name == level_name)
      model->draw(scene, editor_models, meters_per_pixel);
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

void Scenario::sim_tick(Building& building)
{
  // see if we need to start any new model behaviors
  for (auto& schedule_item : behavior_schedule)
  {
    if (schedule_item.started)
      continue;
    if (sim_time_seconds > schedule_item.start_seconds)
      start_behavior_schedule_item(schedule_item, building);
  }

  const double dt = 0.01;

  for (auto& signal_name : behavior_signals)
    printf("SIGNAL: [%s]\n", signal_name.c_str());

  // tick all the model behaviors to move them forward one timestep
  std::vector<std::string> all_outbound_signals;

  for (auto& model : models)
  {
    std::vector<std::string> outbound_signals;
    model->tick(dt, building, models, behavior_signals, outbound_signals);

    // append this model's outbound signals to the composite signal vector
    all_outbound_signals.insert(
        all_outbound_signals.end(),
        outbound_signals.begin(),
        outbound_signals.end());
  }

  sim_time_seconds += dt;

  behavior_signals = all_outbound_signals;  // save for next tick

  // now that we have computed all the states, copy them into the
  // state used for rendering
  // todo: mutex
  for (auto& model : models)
    model->state = model->next_state;

  // if we have reached the end of our schedule, reset time and loop
  bool all_started = true;
  for (auto& schedule_item : behavior_schedule)
    if (!schedule_item.started)
    {
      all_started = false;
      break;
    }

  if (all_started)
  {
    bool all_completed = true;
    // see if we have all finished
    for (auto& model : models)
      if (!model->behavior->is_completed())
        all_completed = false;

    if (all_completed)
    {
      sim_time_seconds = 0;
      for (auto& schedule_item : behavior_schedule)
        schedule_item.started = false;
    }
  }
}

void Scenario::sim_reset(Building& building)
{
  sim_time_seconds = 0.0;

  // reset all our models and restore ownership to their building levels
  for (auto& model : models)
    for (auto& level : building.levels)
      if (level->name == model->starting_level)
      {
        level->models.push_back(std::move(model));
        break;
      }
  models.clear();  // this will now just be a bunch of empty unique_ptr
}

void Scenario::start_behavior_schedule_item(
    BehaviorScheduleItem& item,
    Building& building)
{
  printf(
      "Scenario::start_behavior_schedule_item(%s, %s)\n",
      item.model_name.c_str(),
      item.behavior_name.c_str());
  item.start_seconds = sim_time_seconds;
  item.started = true;

  // generate the behavior instance
  std::unique_ptr<Behavior> model_behavior;
  for (const auto& behavior : behaviors)
    if (behavior->name == item.behavior_name)
      model_behavior =
          std::move(behavior->instantiate(item.behavior_params, item.model_name));

  if (!model_behavior)
  {
    printf("couldn't find behavior [%s]!\n", item.behavior_name.c_str());
    return;
  }

  // see if we already own this model
  for (auto& model : models)
    if (model->instance_name == item.model_name)
    {
      model->set_behavior(std::move(model_behavior));
      return;
    }

  // if we get here, we need to go take ownership of the model
  for (auto& level : building.levels)
    for (auto it = level->models.begin(); it != level->models.end(); ++it)
    {
      if ((*it)->instance_name == item.model_name)
      {
        std::lock_guard<std::mutex> building_guard(building.building_mutex);

        (*it)->set_behavior(std::move(model_behavior));
        models.push_back(std::move(*it));
        level->models.erase(it);
        // now that we've called erase(), we need to stop this iteration
        // which is fine, since we only wanted to find the first match.
        return;
      }
    }

  // if we get here, we never found the model name :(
  printf("couldn't find model [%s]\n", item.model_name.c_str());
}

void Scenario::clear_scene()
{
  printf("Scenario::clear_scene()\n");
  for (auto& model : models)
    model->clear_scene();
}
