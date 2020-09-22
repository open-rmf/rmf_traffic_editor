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

#include "model_type_table.h"

using namespace crowd_sim;

//===================================================
std::shared_ptr<ModelTypeTab> ModelTypeTab::init_and_make(
  CrowdSimImplPtr crowd_sim_impl)
{
  const QStringList labels =
  {"name", "animation", "anim_speed",
    "model_uri", "x", "y", "z", "pitch", "roll", "yaw",
    ""};

  auto model_type_tab = std::make_shared<ModelTypeTab>(crowd_sim_impl, labels);
  if (!model_type_tab)
  {
    printf("Failed to create model_type table! Exiting");
    return nullptr;
  }
  model_type_tab->setMinimumSize(1200, 400);
  return model_type_tab;
}

//===================================================
void ModelTypeTab::list_item_in_cache()
{
  auto cache_count = get_cache_size();
  for (auto i = 0; i < cache_count; i++)
  {
    auto current_model_type = _cache[i];
    setItem(i, 0,
      new QTableWidgetItem(QString::fromStdString(
        current_model_type.get_name() )));
    setItem(i, 1,
      new QTableWidgetItem(QString::fromStdString(
        current_model_type.get_animation() )));
    setItem(i, 2,
      new QTableWidgetItem(QString::number(
        current_model_type.get_animation_speed() )));
    setItem(i, 3,
      new QTableWidgetItem(QString::fromStdString(
        current_model_type.get_model_uri() )));
    setItem(i, 4,
      new QTableWidgetItem(QString::number(
        current_model_type.get_init_pose()[0])));
    setItem(i, 5,
      new QTableWidgetItem(QString::number(
        current_model_type.get_init_pose()[1])));
    setItem(i, 6,
      new QTableWidgetItem(QString::number(
        current_model_type.get_init_pose()[2])));
    setItem(i, 7,
      new QTableWidgetItem(QString::number(
        current_model_type.get_init_pose()[3])));
    setItem(i, 8,
      new QTableWidgetItem(QString::number(
        current_model_type.get_init_pose()[4])));
    setItem(i, 9,
      new QTableWidgetItem(QString::number(
        current_model_type.get_init_pose()[5])));
  }
}

//===================================================
void ModelTypeTab::add_button_click()
{
  _cache.emplace_back("new_type", "new_animation");
}

//===================================================
void ModelTypeTab::delete_button_click(size_t row_number)
{
  if (row_number > _cache.size())
    return;
  _cache.erase(_cache.begin() + row_number);
}

//===================================================
void ModelTypeTab::save()
{
  auto row_count = rowCount();
  std::vector<ModelType> tmp_cache;
  bool OK_status;
  for (auto i = 0; i < row_count - 1; i++)
  {
    ModelType current_model_type("", "");
    current_model_type.set_name(
      item(i, 0)->text().toStdString() );
    current_model_type.set_animation(
      item(i, 1)->text().toStdString() );
    current_model_type.set_animation_speed(
      item(i, 2)->text().toDouble(&OK_status) );
    current_model_type.set_model_uri(
      item(i, 3)->text().toStdString() );

    std::vector<double> init_pose = {
      item(i, 4)->text().toDouble(&OK_status),
      item(i, 5)->text().toDouble(&OK_status),
      item(i, 6)->text().toDouble(&OK_status),
      item(i, 7)->text().toDouble(&OK_status),
      item(i, 8)->text().toDouble(&OK_status),
      item(i, 9)->text().toDouble(&OK_status)
    };
    current_model_type.set_init_pose(
      init_pose);

    if (!current_model_type.is_valid())
      continue;
    tmp_cache.push_back(current_model_type);
  }
  _cache = tmp_cache;
}

//===================================================
void ModelTypeTab::save_to_impl()
{
  save();
  get_impl()->save_model_types(_cache);
}