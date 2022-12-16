/*
 * Copyright (C) 2020-2021 Open Source Robotics Foundation
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

#include <iostream>

#include <QString>

#include "crowd_sim_editor_table.h"
#include "crowd_sim_dialog.h"

using namespace crowd_sim;

//=====================================================================
CrowdSimEditorTable::CrowdSimEditorTable(const Building& building)
: TableList(3),
  _building(building)
{
  if (_building.crowd_sim_impl == nullptr)
  {
    printf("Initialize crowd_sim_implementation for building\n");
    _impl = std::make_shared<crowd_sim::CrowdSimImplementation>();
    _building.crowd_sim_impl = _impl;
  }
  else
  {
    _impl = _building.crowd_sim_impl;
  }

  const QStringList labels = { "Name", "Status", "" };
  setHorizontalHeaderLabels(labels);
  setRowCount(
    _reserved_rows +  // checkbox for enable_crowd_sim, LineEdit for update_time_step
    _required_components.size());

  _enable_crowd_sim_name_item =
    new QTableWidgetItem(QString::fromStdString("enable_crowd_sim"));
  setItem(0, 0, _enable_crowd_sim_name_item);
  _enable_crowd_sim_checkbox = new QCheckBox(this);
  _enable_crowd_sim_checkbox->setChecked(_impl->get_enable_crowd_sim());
  setCellWidget(0, 1, _enable_crowd_sim_checkbox);
  connect(
    _enable_crowd_sim_checkbox,
    &QAbstractButton::clicked,
    [this](bool box_checked)
    {
      _impl->set_enable_crowd_sim(box_checked);
    }
  );

  _update_time_step_name_item =
    new QTableWidgetItem(QString::fromStdString("update_time_step"));
  setItem(1, 0, _update_time_step_name_item);
  _update_time_step_value_item =
    new QLineEdit(QString::number(_impl->get_update_time_step()), this);
  setCellWidget(1, 1, _update_time_step_value_item);
  connect(
    _update_time_step_value_item,
    &QLineEdit::editingFinished,
    [this]()
    {
      bool OK_status;
      double update_time_step =
      _update_time_step_value_item->text().toDouble(&OK_status);
      _impl->set_update_time_step(
        OK_status ? update_time_step : 0.1);
    }
  );

  update();

  for (size_t i = 0; i < _required_components.size(); ++i)
  {
    int row_id = _reserved_rows + i;

    QTableWidgetItem* name_item =
      new QTableWidgetItem(QString::fromStdString(_required_components[i]) );
    setItem(row_id, 0, name_item);
    QPushButton* edit_button = new QPushButton("Edit", this);
    setCellWidget(row_id, 2, edit_button);
    edit_button->setStyleSheet("QTableWidgetItem { background-color: red; }");

    connect(
      edit_button,
      &QAbstractButton::clicked,
      [this, i]()
      {
        update();
        CrowdSimDialog dialog(_impl, _required_components[i]);
        dialog.exec();
        update();
      }
    );
  }

}

//=================================================
void CrowdSimEditorTable::update()
{
  update_goal_area();
  update_navmesh_level();
  update_external_agent_from_spawn_point();
  update_external_agent_state();

  blockSignals(true);

  _enable_crowd_sim_checkbox->setChecked(_impl->get_enable_crowd_sim() );
  _update_time_step_value_item->setText(QString::number(_impl->
    get_update_time_step() ));

  size_t status_number = 0;
  for (size_t i = 0; i < _required_components.size(); ++i)
  {
    status_number = 0;
    if ("States" == _required_components[i])
    {
      status_number = _impl->get_states().size();
    }
    if ("GoalSets" == this->_required_components[i])
    {
      status_number = _impl->get_goal_sets().size();
    }
    if ("AgentProfiles" == this->_required_components[i])
    {
      status_number = _impl->get_agent_profiles().size();
    }
    if ("Transitions" == this->_required_components[i])
    {
      status_number = _impl->get_transitions().size();
    }
    if ("AgentGroups" == this->_required_components[i])
    {
      status_number = _impl->get_agent_groups().size();
    }
    if ("ModelTypes" == this->_required_components[i])
    {
      status_number = _impl->get_model_types().size();
    }

    setItem(
      _reserved_rows + i,
      1,
      new QTableWidgetItem(QString::number(status_number)));
  }

  blockSignals(false);
}

//===================================================
void CrowdSimEditorTable::update_goal_area()
{
  _goal_areas_cache.clear();
  for (auto level : _building.levels)
  {
    auto vertex_list = level.vertices;
    for (auto vertex : vertex_list)
    {
      if (vertex.params.find("human_goal_set_name") == vertex.params.end() )
        continue;
      auto param = vertex.params["human_goal_set_name"];
      if (param.type != param.STRING)
      {
        std::cout << "Error param type for human_goal_set_name." << std::endl;
        return;
      }
      _goal_areas_cache.insert(param.value_string);
    }
  }
  _impl->set_goal_areas(_goal_areas_cache);
}

//====================================================
void CrowdSimEditorTable::update_navmesh_level()
{
  _navmesh_filename_cache.clear();
  for (auto level : _building.levels)
  {
    _navmesh_filename_cache.emplace_back(level.name + "_navmesh.nav");
  }
  _impl->set_navmesh_file_name(_navmesh_filename_cache);
}

//====================================================
void CrowdSimEditorTable::update_external_agent_from_spawn_point()
{
  std::vector<std::string> spawn_point_name;

  for (auto level : _building.levels)
  {
    for (auto vertex : level.vertices)
    {
      if (vertex.params.find("spawn_robot_name") != vertex.params.end())
      {
        spawn_point_name.emplace_back(
          vertex.params["spawn_robot_name"].value_string);
      }
    }
  }

  auto agent_groups = _impl->get_agent_groups();
  if (agent_groups.size() == 0)
  {
    agent_groups.emplace_back(0, true);
  }
  auto& external_group = agent_groups.at(0);
  external_group.set_external_agent_name(spawn_point_name);
  _impl->save_agent_groups(agent_groups);
}

//========================================================
void CrowdSimEditorTable::update_external_agent_state()
{
  auto states = _impl->get_states();
  if (states.size() == 0)
  {
    states.emplace_back("external_static");
  }
  auto& external_state = states.at(0);
  external_state.set_name("external_static");
  external_state.set_final_state(true);
  _impl->save_states(states);
}
