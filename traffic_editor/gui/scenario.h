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

#ifndef SCENARIO_H
#define SCENARIO_H

#include "traffic_editor/building.h"
#include "traffic_editor/editor_model.h"
#include "traffic_editor/model.h"
#include "scenario_level.h"
#include "traffic_editor/vertex.h"
#include "plugins/simulation.h"

#ifdef HAS_IGNITION_PLUGIN
#include <ignition/plugin/SpecializedPluginPtr.hh>
#include <ignition/plugin/Loader.hh>
#endif

#include <map>
#include <memory>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

class Scenario
{
public:
  std::string name;
  std::string filename;
  std::vector<ScenarioLevel> levels;

  /////////////////////////////////
  Scenario();
  ~Scenario();

  bool load();
  bool save() const;

  void clear_scene();

  void draw(
    QGraphicsScene* scene,
    const std::string& level_name,
    const double meters_per_pixel,
    std::vector<EditorModel>& editor_models) const;

  void add_vertex(
    const std::string& level_name,
    const double x,
    const double y);

  void clear_selection(const std::string& level_name);
  bool delete_selected(const std::string& level_name);

  void print() const;

  // simulation stuff
  double sim_time_seconds = 0.0;
  int sim_tick_counter = 0;

  void sim_tick(Building& building);
  void sim_reset(Building& building);

  void scene_update(
    QGraphicsScene* scene,
    Building& building,
    const int level_idx);

  std::vector<std::string> behavior_signals;

#ifdef HAS_IGNITION_PLUGIN
  ignition::plugin::SpecializedPluginPtr<Simulation> sim_plugin;
#endif
};

#endif
