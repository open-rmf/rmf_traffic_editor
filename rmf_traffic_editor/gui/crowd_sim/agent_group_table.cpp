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
#include "agent_group_table.h"


using namespace crowd_sim;

//=======================================
std::shared_ptr<AgentGroupTab> AgentGroupTab::init_and_make(
  CrowdSimImplPtr crowd_sim_impl)
{
  const QStringList labels =
  { "id", "profile", "initial state", "spawn number",
    "external agent", "point_x", "point_y", ""};

  auto agent_group_tab =
    std::make_shared<AgentGroupTab>(crowd_sim_impl, labels);
  if (!agent_group_tab)
  {
    printf("Failed to create agent_group table! Exiting");
    return nullptr;
  }
  agent_group_tab->setMinimumSize(1200, 400);
  return agent_group_tab;
}

//=======================================
void AgentGroupTab::add_button_click()
{
  _cache.emplace_back(get_cache_size());
}

//=======================================
void AgentGroupTab::delete_button_click(size_t row_number)
{
  if (row_number == 0)
  {
    std::cout <<
      "Default agent_group for external agents is not allowed to be deleted. "
              << std::endl;
    return;
  }
  if (row_number > _cache.size())
    return;
  _cache.erase(_cache.begin() + row_number);
}

//=======================================
void AgentGroupTab::list_item_in_cache()
{
  auto cache_count = get_cache_size();

  for (auto i = 0; i < cache_count; i++)
  {
    auto current_group = _cache[i];

    setItem(i, 0,
      new QTableWidgetItem(QString::number(
        static_cast<int>(current_group.get_group_id()))));

    auto current_profile = current_group.get_agent_profile();
    if (i == 0)
    {
      setItem(i, 1,
        new QTableWidgetItem(QString::fromStdString(current_profile) ) );
    }
    else
    {
      QComboBox* profile_combo = new QComboBox;
      _add_profiles_in_combobox(profile_combo, current_profile);
      setCellWidget(i, 1, profile_combo);
    }

    auto current_state = current_group.get_initial_state();
    if (i == 0)
    {
      setItem(i, 2,
        new QTableWidgetItem(QString::fromStdString(current_state) ) );
    }
    else
    {
      QComboBox* state_combo = new QComboBox;
      _add_states_in_combobox(state_combo, current_state);
      setCellWidget(i, 2, state_combo);
    }

    setItem(i, 3,
      new QTableWidgetItem(QString::number(current_group.get_spawn_number())));

    std::string external_agent_name = "";
    if (current_group.is_external_group())
    {
      for (auto name : current_group.get_external_agent_name())
      {
        external_agent_name += name + ";";
      }
    }
    setItem(i, 4,
      new QTableWidgetItem(QString::fromStdString(external_agent_name) ));

    auto spawn_point = current_group.get_spawn_point();
    setItem(i, 5,
      new QTableWidgetItem(QString::number(spawn_point.first)));

    setItem(i, 6,
      new QTableWidgetItem(QString::number(spawn_point.second)));
  }
}

//=======================================
void AgentGroupTab::save()
{
  auto row_count = rowCount();
  std::vector<AgentGroup> tmp_cache;
  bool OK_status;
  for (auto row = 0; row < row_count-1; row++)
  {
    if (row == 0)
    {
      tmp_cache.push_back(_cache[0]);
      continue;
    }

    AgentGroup current_group(row);

    auto profile_combo = static_cast<QComboBox*>(cellWidget(row, 1));
    current_group.set_agent_profile(profile_combo->currentText().toStdString());

    auto state_combo = static_cast<QComboBox*>(cellWidget(row, 2));
    current_group.set_initial_state(state_combo->currentText().toStdString());

    int spawn_number = item(row, 3)->text().toInt(&OK_status);
    if (OK_status)
    {
      current_group.set_spawn_number(spawn_number);
    }
    else
    {
      std::cout << "Invalid input of spawn_number, use default 0." << std::endl;
    }

    double point_x = item(row, 5)->text().toDouble(&OK_status);
    if (!OK_status)
    {
      std::cout <<
        "Invalid input of x for spawn point coord, use default 0.0." <<
        std::endl;
      point_x = 0.0;
    }
    double point_y = item(row, 6)->text().toDouble(&OK_status);
    if (!OK_status)
    {
      std::cout <<
        "Invalid input of y for spawn point coord, use default 0.0." <<
        std::endl;
      point_y = 0.0;
    }
    current_group.set_spawn_point(point_x, point_y);
    tmp_cache.push_back(current_group);
  }
  _cache = tmp_cache;
}

//=======================================
void AgentGroupTab::save_to_impl()
{
  save();
  get_impl()->save_agent_groups(_cache);
}

//=======================================
void AgentGroupTab::_add_profiles_in_combobox(
  QComboBox* profile_combo,
  std::string current_profile)
{
  for (auto profile : get_impl()->get_agent_profiles())
  {
    profile_combo->addItem(QString::fromStdString(profile.profile_name));
  }
  int current_index =
    profile_combo->findText(QString::fromStdString(current_profile));
  profile_combo->setCurrentIndex(current_index >= 0 ? current_index : 0);
}

//=======================================
void AgentGroupTab::_add_states_in_combobox(
  QComboBox* state_combo,
  std::string current_state)
{
  for (auto state : get_impl()->get_states())
  {
    state_combo->addItem(QString::fromStdString(state.get_name()));
  }
  int current_index = state_combo->findText(QString::fromStdString(
        current_state));
  state_combo->setCurrentIndex(current_index >= 0 ? current_index : 0);
}