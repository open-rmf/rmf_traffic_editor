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

#include "profile_model_type_layout.h"
#include "table_list.h"
#include <QTabWidget>
#include <QLineEdit>
#include <QLabel>

using namespace crowd_sim;

//===================================================
std::shared_ptr<ProfileModelTypeLayout> ProfileModelTypeLayout::init_and_make(
  CrowdSimImplPtr crowd_sim_impl)
{
  auto profile_model_type_layout = std::make_shared<ProfileModelTypeLayout>(
    crowd_sim_impl);
  if (!profile_model_type_layout)
  {
    printf("Failed to create profile_model_type layout! Exiting");
    return nullptr;
  }
  return profile_model_type_layout;
}

//===================================================
void ProfileModelTypeLayout::list_item_in_cache()
{
  blockSignals(true);
  for (auto& profile:_profile_cache)
  {
    _list_widget->addItem(QString::fromStdString(profile.profile_name));
  }
  blockSignals(false);
}

//===================================================
void ProfileModelTypeLayout::load_item_name()
{
  auto current_profile = _profile_cache[_current_index];
  if (current_profile.profile_name == "external_agent")
    _name_value->setReadOnly(true);
  else
    _name_value->setReadOnly(false);

  _name_value->setText(QString::fromStdString(current_profile.profile_name));
}

void ProfileModelTypeLayout::update_item_detail()
{
  load_item_name();
  load_item_profile();
  load_item_model();
}

void ProfileModelTypeLayout::load_profile_structure()
{
  const QStringList profile_labels =
  { "class", "max_accel", "max_angle_vel", "max_neighbors",
    "max_speed", "neighbor_dist", "obstacle_set", "pref_speed", "r",
    "ORCA_tau", "ORCA_tauObst"};

  const QStringList labels = {"Property", "Value"};

  if (!_profile_table)
    _profile_table = new TableList;
  _profile_table->setHorizontalHeaderLabels(labels);
  _profile_table->setRowCount(profile_labels.size());
  int row_idx = 0;
  for (auto& prop:profile_labels)
  {
    QTableWidgetItem* label_item = new QTableWidgetItem(prop);
    label_item->setFlags(Qt::NoItemFlags);
    _profile_table->setItem(row_idx, 0, label_item);
    row_idx++;
  }
  _profile_table->setMinimumSize(700, 550);
  _tab_widget->addTab(_profile_table, "Profile");
}

//===================================================
void ProfileModelTypeLayout::load_item_profile()
{
  auto current_profile = _profile_cache[_current_index];
  std::vector<QString> profile_values;
  profile_values.push_back(QString::number(static_cast<uint>(current_profile.
    profile_class)));
  profile_values.push_back(QString::number(current_profile.max_accel));
  profile_values.push_back(QString::number(current_profile.max_angle_vel));
  profile_values.push_back(QString::number(static_cast<uint>(current_profile.
    max_neighbors)));
  profile_values.push_back(QString::number(current_profile.max_speed));
  profile_values.push_back(QString::number(current_profile.neighbor_dist));
  profile_values.push_back(QString::number(static_cast<uint>(current_profile.
    obstacle_set)));
  profile_values.push_back(QString::number(current_profile.pref_speed));
  profile_values.push_back(QString::number(current_profile.r));
  profile_values.push_back(QString::number(current_profile.ORCA_tau));
  profile_values.push_back(QString::number(current_profile.ORCA_tauObst));
  int row_idx = 0;
  for (auto& profile_value:profile_values)
  {
    QTableWidgetItem* item = new QTableWidgetItem(profile_value);
    if (current_profile.profile_name == "external_agent")
      item->setFlags(Qt::NoItemFlags);

    _profile_table->setItem(row_idx, 1, item);
    row_idx++;
  }
}

void ProfileModelTypeLayout::load_model_structure()
{
  const QStringList model_type_labels =
  {"animation", "anim_speed"};

  const QStringList labels = {"Property", "Value"};

  auto current_profile = _profile_cache[_current_index];

  if (!_model_type_table)
  {
    _model_type_table = new TableList;
    _model_type_table->setHorizontalHeaderLabels(labels);
    _model_type_table->setRowCount(model_type_labels.size());
    int row_idx = 0;
    for (auto& prop:model_type_labels)
    {
      QTableWidgetItem* label_item = new QTableWidgetItem(prop);
      label_item->setFlags(Qt::NoItemFlags);
      _model_type_table->setItem(row_idx, 0, label_item);
      row_idx++;
    }

    _model_type_table->setMinimumSize(700, 550);
    _tab_widget->addTab(_model_type_table, "Model Type");
  }
}

//===================================================
void ProfileModelTypeLayout::load_item_model()
{
  auto current_profile = _profile_cache[_current_index];
  if (current_profile.profile_name != "external_agent")
  {
    load_model_structure();
    auto current_model_type = _model_cache[_current_index-1];
    _model_type_table->setItem(0, 1,
      new QTableWidgetItem(QString::fromStdString(current_model_type.
      get_animation())));
    _model_type_table->setItem(1, 1,
      new QTableWidgetItem(QString::number(current_model_type.
      get_animation_speed())));
  }
  else
  {
    if (_model_type_table)
    {
      _tab_widget->removeTab(1);
      delete _model_type_table;
      _model_type_table = nullptr;
    }
  }
}

void ProfileModelTypeLayout::load_name_structure()
{
  if (!_name_layout)
  {
    _name_layout = new QHBoxLayout();
    QLabel* name_label = new QLabel();
    name_label->setText("Name:");
    _name_layout->addWidget(name_label);
  }

  if (!_name_value)
  {
    _name_value = new QLineEdit();
    _name_layout->addWidget(_name_value);
    _right_panel->addLayout(_name_layout);
  }

  connect(_name_value, &QLineEdit::editingFinished, [this]()
    {
      if (_current_index > 0)
      {
        auto& current_profile = _profile_cache[_current_index];
        auto& current_model = _model_cache[_current_index - 1];

        auto profile_name = _name_value->text().toStdString();

        if (profile_name == "")
        {
          std::cout << "Error in saving profile_name for Agent Profile: ["
                    << current_profile.profile_name << "], value remains as default." << std::endl;
        }
        else
        {
          current_profile.profile_name = profile_name;
          current_model.set_name(profile_name);
          QListWidgetItem* item_name = _list_widget->item(_current_index);
          item_name->setText(QString::fromStdString(
            current_profile.profile_name));
        }
      }
    });
}

//===================================================
void ProfileModelTypeLayout::initialise_item_detail()
{
  const QStringList labels = {"Property", "Value"};

  if (!_tab_widget)
    _tab_widget = new QTabWidget();


  load_name_structure();
  load_item_name();

  load_profile_structure();
  load_item_profile();

  load_item_model();

  _right_panel->addWidget(_tab_widget);
}

//===================================================
void ProfileModelTypeLayout::add_button_click()
{
  std::string new_type_str = "new type";
  AgentProfile new_profile(new_type_str);
  _profile_cache.push_back(new_profile);
  _model_cache.emplace_back(new_type_str, "new_animation");
  _list_widget->addItem(QString::fromStdString(new_type_str));
  uint current_index = _profile_cache.size()-1;
  _list_widget->setCurrentRow(current_index); // set to latest row and invoke QListWidget::currentRowChanged
}

//===================================================
void ProfileModelTypeLayout::delete_button_click()
{
  if (_current_index > 0)
  {
    _to_delete = true;
    _profile_cache.erase(_profile_cache.begin() + _current_index);
    _model_cache.erase(_model_cache.begin() + _current_index-1);
    delete _list_widget->item(_current_index);
    uint last_index = _profile_cache.size() - 1;
    if (_current_index > last_index)
      _current_index = last_index;// move up
    _to_delete = false;
    update();
  }
}

//===================================================
void ProfileModelTypeLayout::save()
{
  if (_model_type_table && _current_index > 0)
  {
    auto& current_model_type = _model_cache[_current_index-1]; // 1 index lagging behind profile due to external_static excluded
    bool OK_status;
    current_model_type.set_animation(_model_type_table->item(0,
      1)->text().toStdString());
    current_model_type.set_animation_speed(_model_type_table->item(1,
      1)->text().toDouble(&OK_status));
  }

  if (_profile_table)
  {
    auto& current_profile = _profile_cache[_current_index];
    bool OK_status;
    QTableWidgetItem* pItem;

    auto profile_name =
      _list_widget->item(_current_index)->text().toStdString();

    pItem = _profile_table->item(0, 1);
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

    pItem = _profile_table->item(1, 1);
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

    pItem = _profile_table->item(2, 1);
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

    pItem = _profile_table->item(3, 1);
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

    pItem = _profile_table->item(4, 1);
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

    pItem = _profile_table->item(5, 1);
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

    pItem = _profile_table->item(6, 1);
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

    pItem = _profile_table->item(7, 1);
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

    pItem = _profile_table->item(8, 1);
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

    pItem = _profile_table->item(9, 1);
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

    pItem = _profile_table->item(10, 1);
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
  }
}

//===================================================
void ProfileModelTypeLayout::save_to_impl()
{
  save();
  get_impl()->save_agent_profiles(_profile_cache);
  get_impl()->save_model_types(_model_cache);
}