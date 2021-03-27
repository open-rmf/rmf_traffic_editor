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

#include <map>

#include <QWidget>
#include <QComboBox>

#include "to_state_table.h"

//=====================================================
std::shared_ptr<ToStateTab> ToStateTab::init_and_make(
  CrowdSimImplPtr crowd_sim_impl,
  crowd_sim::Transition& transition)
{
  const QStringList labels = {"To State Name", "Weight", ""};

  auto to_state_tab_ptr = std::make_shared<ToStateTab>(crowd_sim_impl,
      transition, labels);
  if (!to_state_tab_ptr)
  {
    printf("Failed to create goal set table! Exiting");
    return nullptr;
  }
  return to_state_tab_ptr;
}

//=====================================================
void ToStateTab::list_item_in_cache()
{
  auto cache_count = get_cache_size();
  for (auto i = 0; i < cache_count; i++)
  {
    auto to_state = _cache.at(i);
    auto to_state_name = to_state.first;
    auto to_state_weight = to_state.second;

    QComboBox* state_comboBox = new QComboBox;
    for (auto state : get_impl()->get_states())
    {
      state_comboBox->addItem(QString::fromStdString(state.get_name() ));
    }
    auto index =
      state_comboBox->findText(QString::fromStdString(to_state_name) );
    state_comboBox->setCurrentIndex(index >= 0 ? index : 0);
    setCellWidget(i, 0, state_comboBox);

    setItem(
      i, 1, new QTableWidgetItem(QString::number(to_state_weight)));
  }
}

//=====================================================
void ToStateTab::add_button_click()
{
  _cache.emplace_back(std::make_pair("", 1));
}

//=====================================================
void ToStateTab::delete_button_click(size_t row_number)
{
  if (row_number > _cache.size())
    return;
  _cache.erase(_cache.begin() + row_number);
}

//=====================================================
void ToStateTab::save()
{
  auto tmp_cache = _cache;
  tmp_cache.clear();
  for (auto i = 0; i < rowCount() - 1; i++)
  {
    QComboBox* current_to_state_combo =
      static_cast<QComboBox*>(cellWidget(i, 0));
    std::string to_state_name =
      current_to_state_combo->currentText().toStdString();

    QTableWidgetItem* weight_item = item(i, 1);
    bool OK_status;
    double to_state_weight = weight_item->text().toDouble(&OK_status);
    if (!OK_status)
    {
      std::cout << "Error in saving ToStateTab, invalid weight provided!" <<
        std::endl;
      continue;
    }
    tmp_cache.emplace_back(std::make_pair(to_state_name, to_state_weight));
  }
  _cache = tmp_cache;
}

//=====================================================
void ToStateTab::save_to_impl()
{
  save();
  _current_transition.clear_to_state();
  for (auto to_state : _cache)
  {
    _current_transition.add_to_state(to_state.first, to_state.second);
  }
}
