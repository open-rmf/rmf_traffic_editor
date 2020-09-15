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

#include "goal_set_table.h"
#include "multi_select_combo_box.h"

using namespace crowd_sim;

//======================================================
std::shared_ptr<GoalSetTab> GoalSetTab::init_and_make(
  CrowdSimImplPtr crowd_sim_impl)
{

  const QStringList labels =
  { "Id", "Goal Area", "Capacity", ""};

  auto goal_set_tab_ptr = std::make_shared<GoalSetTab>(crowd_sim_impl, labels);
  if (!goal_set_tab_ptr)
  {
    printf("Failed to create goal set table! Exiting");
    return nullptr;
  }
  return goal_set_tab_ptr;
}

//======================================================
void GoalSetTab::list_item_in_cache()
{
  auto cache_count = _cache.size();
  for (size_t i = 0; i < cache_count; i++)
  {
    auto& goal_set = _cache[i];
    QTableWidget::setItem(
      i,
      0,
      new QTableWidgetItem(
        QString::number(static_cast<int>(goal_set.get_goal_set_id() ))));

    MultiSelectComboBox* multi_combo_box =
      new MultiSelectComboBox(get_impl()->get_goal_areas());
    multi_combo_box->showCheckedItem(goal_set.get_goal_areas());
    QTableWidget::setCellWidget(
      i,
      1,
      multi_combo_box);

    QTableWidget::setItem(
      i,
      2,
      new QTableWidgetItem(
        QString::number(static_cast<int>(goal_set.get_capacity() ))));
  }
}

//======================================================
// this function will save all the goal_set listed in the table widget into _cache,
// and reassign goal_set_id as the row_number
void GoalSetTab::save()
{
  auto row_count = rowCount();
  std::vector<GoalSet> tmp_cache;

  for (auto i = 0; i < row_count - 1; i++)
  {
    auto pItem_setid = item(i, 0);
    bool OK_status;
    auto set_id = pItem_setid->text().toInt(&OK_status);
    if (!OK_status)
    {
      std::cout << "Invalid goal set id in row " << i << " with [" <<
        pItem_setid->text().toStdString() << "]" << std::endl;
      return;
    }
    if (set_id != i)
    {
      std::cout << "Reassign goal set id for row" << i << std::endl;
    }

    auto pItem_areas = static_cast<MultiSelectComboBox*>(cellWidget(i, 1));
    //if no goal area selected, don't save the goal set
    if (pItem_areas->getCheckResult().empty())
      continue;

    auto pItem_capacity = item(i, 2);
    auto capacity = pItem_capacity->text().toInt(&OK_status);
    if (!OK_status)
    {
      std::cout << "Invalid capacity for goal set" << std::endl;
      return;
    }

    auto cur_it =
      tmp_cache.emplace(tmp_cache.end(), static_cast<size_t>(set_id));
    for (auto item : pItem_areas->getCheckResult() )
    {
      cur_it->add_goal_area(item);
    }
    cur_it->set_capacity(static_cast<size_t>(capacity));
  }
  _cache = tmp_cache;
}

//======================================================
void GoalSetTab::save_to_impl()
{
  save();
  get_impl()->save_goal_sets(_cache);
}

//======================================================
void GoalSetTab::add_button_click()
{
  _cache.emplace_back(_cache.size());
}

//======================================================
void GoalSetTab::delete_button_click(size_t row_number)
{
  if (row_number >= _cache.size())
    return;
  _cache.erase(_cache.begin() + row_number);
}
