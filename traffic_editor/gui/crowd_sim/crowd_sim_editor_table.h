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

#ifndef CROWD_SIM_EDITOR_TABLE__H
#define CROWD_SIM_EDITOR_TABLE__H

#include <vector>
#include <string>
#include <set>

#include <QTableWidget>
#include <QtWidgets>

#include "table_list.h"
#include "building.h"
#include <traffic_editor/crowd_sim/crowd_sim_impl.h>

using namespace crowd_sim;

class CrowdSimEditorTable : public TableList
{
  Q_OBJECT;

public:
  CrowdSimEditorTable(Building& building);
  ~CrowdSimEditorTable() {}

  void update(int level = 0);
  void update_goal_area();
  void set_navmesh_file_name()
  {
    _impl->set_navmesh_file_name(_navmesh_filename);
  }
  void update_external_agent_state();
  void setParentTabWidget(QTabWidget* parentwidget)
  {
    _parentwidget = parentwidget;
  }
  int getCurrentLevel() {return _level;}
private:
  Building& _building;
  CrowdSimImplPtr _impl;

  int _level;
  // reserved rows for checkbox for enable_crowd_sim, LineEdit for updtae_time_step
  int _reserved_rows = 3;
  std::vector<std::string> _required_components {
    "GoalSets",
    "States",
    "Transitions",
    "ProfileModelTypes"
  };
  std::set<std::string> _goal_areas_cache;

  std::string _navmesh_filename;
  QTableWidgetItem* _enable_crowd_sim_name_item;
  QCheckBox* _enable_crowd_sim_checkbox;
  QTableWidgetItem* _update_time_step_name_item;
  QLineEdit* _external_agent_value_item;
  QLineEdit* _update_time_step_value_item;
  QTabWidget* _parentwidget;
};

#endif
