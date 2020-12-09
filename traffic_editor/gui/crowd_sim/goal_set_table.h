/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifndef CROWD_SIM_GOAL_SET_TABLE__H
#define CROWD_SIM_GOAL_SET_TABLE__H

#include <memory>
#include <vector>

#include <traffic_editor/crowd_sim/crowd_sim_impl.h>
#include <traffic_editor/crowd_sim/goal_set.h>

#include "crowd_sim_table_base.h"

using namespace crowd_sim;

class GoalSetTab : public CrowdSimTableBase
{
public:
  static std::shared_ptr<GoalSetTab> init_and_make(
    CrowdSimImplPtr crowd_sim_impl);

  GoalSetTab(CrowdSimImplPtr crowd_sim_impl, const QStringList& labels)
  : CrowdSimTableBase(crowd_sim_impl, labels)
  {
    _cache = get_impl()->get_goal_sets();
  }
  ~GoalSetTab() {}

  int get_cache_size() const override
  {
    return static_cast<int>(_cache.size());
  }

  void list_item_in_cache() override;
  void save() override;
  void save_to_impl() override;
  void add_button_click() override;
  void delete_button_click(size_t row_number) override;

private:
  std::vector<crowd_sim::GoalSet> _cache;
};

#endif