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

#include <iostream>
#include <vector>

#include "agent_profile_table.h"

using namespace crowd_sim;

//===================================================
std::shared_ptr<AgentProfileTab> AgentProfileTab::init_and_make(
  CrowdSimImplPtr crowd_sim_impl)
{
  const QStringList labels =
  { "Name", "class", "max_accel", "max_angle_vel", "max_neighbors",
    "max_speed", "neighbor_dist", "obstacle_set", "pref_speed", "r",
    "ORCA_tau", "ORCA_tauObst", ""};

  auto agent_profile_tab = std::make_shared<AgentProfileTab>(crowd_sim_impl,
      labels);
  if (!agent_profile_tab)
  {
    printf("Failed to create agent_profile table! Exiting");
    return nullptr;
  }
  agent_profile_tab->setMinimumSize(1600, 400);
  return agent_profile_tab;
}

//===================================================
void AgentProfileTab::list_item_in_cache()
{
  auto cache_count = get_cache_size();
  for (auto i = 0; i < cache_count; i++)
  {
    auto current_profile = _cache[i];
    setItem(i, 0,
      new QTableWidgetItem(QString::fromStdString(
        current_profile.profile_name)));
    setItem(i, 1,
      new QTableWidgetItem(QString::number(
        static_cast<uint>(current_profile.profile_class))));
    setItem(i, 2,
      new QTableWidgetItem(QString::number(current_profile.max_accel)));
    setItem(i, 3,
      new QTableWidgetItem(QString::number(current_profile.max_angle_vel)));
    setItem(i, 4,
      new QTableWidgetItem(QString::number(
        static_cast<uint>(current_profile.max_neighbors))));
    setItem(i, 5,
      new QTableWidgetItem(QString::number(current_profile.max_speed)));
    setItem(i, 6,
      new QTableWidgetItem(QString::number(current_profile.neighbor_dist)));
    setItem(i, 7,
      new QTableWidgetItem(QString::number(
        static_cast<uint>(current_profile.obstacle_set))));
    setItem(i, 8,
      new QTableWidgetItem(QString::number(current_profile.pref_speed)));
    setItem(i, 9,
      new QTableWidgetItem(QString::number(current_profile.r)));
    setItem(i, 10,
      new QTableWidgetItem(QString::number(current_profile.ORCA_tau)));
    setItem(i, 11,
      new QTableWidgetItem(QString::number(current_profile.ORCA_tauObst)));
  }
}

//===================================================
void AgentProfileTab::add_button_click()
{
  _cache.emplace_back("new profile");
}

//===================================================
void AgentProfileTab::delete_button_click(size_t row_number)
{
  if (row_number == 0)
  {
    std::cout <<
      "Default external agnet profile is not allowed to be deleted." <<
      std::endl;
    return;
  }
  if (row_number > _cache.size())
    return;
  _cache.erase(_cache.begin() + row_number);
}

//===================================================
void AgentProfileTab::save_to_impl()
{
  save();
  get_impl()->save_agent_profiles(_cache);
}

//===================================================
void AgentProfileTab::save()
{
  auto row_count = rowCount();
  std::vector<AgentProfile> tmp_cache;

  QTableWidgetItem* pItem;
  bool OK_status;
  for (auto i = 0; i < row_count - 1; i++)
  {
    AgentProfile current_profile("");

    pItem = item(i, 0);
    auto profile_name = pItem->text().toStdString();
    current_profile.profile_name = profile_name;

    pItem = item(i, 1);
    auto profile_class = pItem->text().toInt(&OK_status);
    if (OK_status)
    {
      current_profile.profile_class = static_cast<size_t>(profile_class);
    }
    else
    {
      std::cout << "Error in saving profile_class for Agent Profile: ["
                << profile_name << "], value remains as default." << std::endl;
    }

    pItem = item(i, 2);
    auto max_accel = pItem->text().toDouble(&OK_status);
    if (OK_status)
    {
      current_profile.max_accel = static_cast<double>(max_accel);
    }
    else
    {
      std::cout << "Error in saving max_accel for Agent Profile: ["
                << profile_name << "]" << std::endl;
    }

    pItem = item(i, 3);
    auto max_angle_vel = pItem->text().toDouble(&OK_status);
    if (OK_status)
    {
      current_profile.max_angle_vel = static_cast<double>(max_angle_vel);
    }
    else
    {
      std::cout << "Error in saving max_angle_vel for Agent Profile: ["
                << profile_name << "]" << std::endl;
    }

    pItem = item(i, 4);
    auto max_neighbors = pItem->text().toInt(&OK_status);
    if (OK_status)
    {
      current_profile.max_neighbors = static_cast<size_t>(max_neighbors);
    }
    else
    {
      std::cout << "Error in saving max_neighbors for Agent Profile: ["
                << profile_name << "]" << std::endl;
    }

    pItem = item(i, 5);
    auto max_speed = pItem->text().toDouble(&OK_status);
    if (OK_status)
    {
      current_profile.max_speed = static_cast<double>(max_speed);
    }
    else
    {
      std::cout << "Error in saving max_speed for Agent Profile: ["
                << profile_name << "]" << std::endl;
    }

    pItem = item(i, 6);
    auto neighbor_dist = pItem->text().toDouble(&OK_status);
    if (OK_status)
    {
      current_profile.neighbor_dist = static_cast<double>(neighbor_dist);
    }
    else
    {
      std::cout << "Error in saving neighbor dist for Agent Profile: ["
                << profile_name << "]" << std::endl;
    }

    pItem = item(i, 7);
    auto obstacle_set = pItem->text().toInt(&OK_status);
    if (OK_status)
    {
      current_profile.obstacle_set = static_cast<double>(obstacle_set);
    }
    else
    {
      std::cout << "Error in saving obstacle_set for Agent Profile: ["
                << profile_name << "]" << std::endl;
    }

    pItem = item(i, 8);
    auto pref_speed = pItem->text().toDouble(&OK_status);
    if (OK_status)
    {
      current_profile.pref_speed = static_cast<double>(pref_speed);
    }
    else
    {
      std::cout << "Error in saving pref_speed for Agent Profile: ["
                << profile_name << "]" << std::endl;
    }

    pItem = item(i, 9);
    auto r = pItem->text().toDouble(&OK_status);
    if (OK_status)
    {
      current_profile.r = static_cast<double>(r);
    }
    else
    {
      std::cout << "Error in saving r for Agent Profile: ["
                << profile_name << "]" << std::endl;
    }

    pItem = item(i, 10);
    auto ORCA_tau = pItem->text().toDouble(&OK_status);
    if (OK_status)
    {
      current_profile.ORCA_tau = static_cast<double>(ORCA_tau);
    }
    else
    {
      std::cout << "Error in saving ORCA_tau for Agent Profile: ["
                << profile_name << "]" << std::endl;
    }

    pItem = item(i, 11);
    auto ORCA_tauObst = pItem->text().toDouble(&OK_status);
    if (OK_status)
    {
      current_profile.ORCA_tauObst = static_cast<double>(ORCA_tauObst);
    }
    else
    {
      std::cout << "Error in saving ORCA_tauObst for Agent Profile: ["
                << profile_name << "]" << std::endl;
    }

    tmp_cache.push_back(current_profile);
  }
  _cache = tmp_cache;
}
