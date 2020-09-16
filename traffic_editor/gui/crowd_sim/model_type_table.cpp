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
    "gazebo_model", "gazebo_idle", "x", "y", "z", "pitch", "roll", "yaw",
    "ign_model", "x", "y", "z", "pitch", "roll", "yaw",
    ""};

  auto model_type_tab = std::make_shared<ModelTypeTab>(crowd_sim_impl, labels);
  if (!model_type_tab)
  {
    printf("Failed to create model_type table! Exiting");
    return nullptr;
  }
  model_type_tab->setMinimumSize(1600, 400);
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
      new QTableWidgetItem(QString::fromStdString(current_model_type.
      get_animation() )));
    setItem(i, 2,
      new QTableWidgetItem(QString::number(current_model_type.
      get_animation_speed() )));
    setItem(i, 3,
      new QTableWidgetItem(QString::fromStdString(current_model_type.
      get_gazebo_conf().filename)));
    setItem(i, 4,
      new QTableWidgetItem(QString::fromStdString(current_model_type.
      get_gazebo_conf().idle_filename)));
    setItem(i, 5,
      new QTableWidgetItem(QString::number(current_model_type.get_gazebo_conf().
      initial_pose[0])));
    setItem(i, 6,
      new QTableWidgetItem(QString::number(current_model_type.get_gazebo_conf().
      initial_pose[1])));
    setItem(i, 7,
      new QTableWidgetItem(QString::number(current_model_type.get_gazebo_conf().
      initial_pose[2])));
    setItem(i, 8,
      new QTableWidgetItem(QString::number(current_model_type.get_gazebo_conf().
      initial_pose[3])));
    setItem(i, 9,
      new QTableWidgetItem(QString::number(current_model_type.get_gazebo_conf().
      initial_pose[4])));
    setItem(i, 10,
      new QTableWidgetItem(QString::number(current_model_type.get_gazebo_conf().
      initial_pose[5])));
    setItem(i, 11,
      new QTableWidgetItem(QString::fromStdString(current_model_type.
      get_ign_conf().filename)));
    setItem(i, 12,
      new QTableWidgetItem(QString::number(current_model_type.get_ign_conf().
      initial_pose[0])));
    setItem(i, 13,
      new QTableWidgetItem(QString::number(current_model_type.get_ign_conf().
      initial_pose[1])));
    setItem(i, 14,
      new QTableWidgetItem(QString::number(current_model_type.get_ign_conf().
      initial_pose[2])));
    setItem(i, 15,
      new QTableWidgetItem(QString::number(current_model_type.get_ign_conf().
      initial_pose[3])));
    setItem(i, 16,
      new QTableWidgetItem(QString::number(current_model_type.get_ign_conf().
      initial_pose[4])));
    setItem(i, 17,
      new QTableWidgetItem(QString::number(current_model_type.get_ign_conf().
      initial_pose[5])));
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

    std::string filename = item(i, 3)->text().toStdString();
    std::string idle_filename = item(i, 4)->text().toStdString();
    std::vector<double> initial_pose = {
      item(i, 5)->text().toDouble(&OK_status),
      item(i, 6)->text().toDouble(&OK_status),
      item(i, 7)->text().toDouble(&OK_status),
      item(i, 8)->text().toDouble(&OK_status),
      item(i, 9)->text().toDouble(&OK_status),
      item(i, 10)->text().toDouble(&OK_status)
    };
    current_model_type.set_gazebo_conf(
      filename,
      idle_filename,
      initial_pose);

    filename = item(i, 11)->text().toStdString();
    initial_pose = {
      item(i, 12)->text().toDouble(&OK_status),
      item(i, 13)->text().toDouble(&OK_status),
      item(i, 14)->text().toDouble(&OK_status),
      item(i, 15)->text().toDouble(&OK_status),
      item(i, 16)->text().toDouble(&OK_status),
      item(i, 17)->text().toDouble(&OK_status)
    };
    current_model_type.set_ign_conf(filename, initial_pose);

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