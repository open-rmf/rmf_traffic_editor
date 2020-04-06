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

#include "behavior.h"
#include "behavior_schedule_item.h"
#include "building.h"
#include "model.h"
#include "scenario_level.h"
#include "vertex.h"

#include <map>
#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>

class Scenario
{
public:
  std::string name;
  std::string filename;
  std::vector<ScenarioLevel> levels;
  std::vector<std::unique_ptr<Behavior> > behaviors;
  std::vector<BehaviorScheduleItem> behavior_schedule;

  // when a model starts to become an "active participant" in a scenario,
  // it is removed from the building level and owned by the Scenario
  std::vector<std::unique_ptr<Model> > models;

  /////////////////////////////////
  Scenario();
  ~Scenario();

  bool load();
  bool save() const;

  void draw(
      QGraphicsScene *scene,
      const std::string& level_name,
      const double meters_per_pixel) const;

  void add_vertex(
      const std::string& level_name,
      const double x,
      const double y);

  void clear_selection(const std::string& level_name);
  bool delete_selected(const std::string& level_name);

  void print() const;

  // simulation stuff
  double sim_time_seconds = 0.0;

  void sim_tick(Building& building);
  void sim_reset(Building& building);

  void start_behavior_schedule_item(
      BehaviorScheduleItem& item,
      Building& building);
};

#endif
