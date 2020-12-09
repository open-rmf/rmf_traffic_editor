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

#ifndef CROWD_SIM_PROFILE_MODEL_TYPE_LAYOUT__H
#define CROWD_SIM_PROFILE_MODEL_TYPE_LAYOUT__H

#include <QWidget>
#include <QComboBox>

#include <traffic_editor/crowd_sim/crowd_sim_impl.h>

#include "crowd_sim_layout_base.h"
#include "table_list.h"

using namespace crowd_sim;

class ProfileModelTypeLayout : public CrowdSimLayoutBase
{
public:

  static std::shared_ptr<ProfileModelTypeLayout> init_and_make(
    CrowdSimImplPtr crowd_sim_impl);

  ProfileModelTypeLayout(CrowdSimImplPtr crowd_sim_impl)
  : CrowdSimLayoutBase(crowd_sim_impl),
    _profile_table(nullptr),
    _model_type_table(nullptr),
    _tab_widget(nullptr),
    _name_layout(nullptr),
    _name_value(nullptr)
  {
    _profile_cache = get_impl()->get_agent_profiles();
    _model_cache = get_impl()->get_model_types();
  }
  ~ProfileModelTypeLayout() {}

  int get_cache_size() const
  {
    assert(_profile_cache.size() == _model_cache.size());
    return static_cast<int>(_profile_cache.size());
  }
  virtual void list_item_in_cache() override;
  virtual void initialise_item_detail() override;
  virtual void update_item_detail() override;
  void load_profile_structure();
  void load_model_structure();
  void load_name_structure();
  void load_item_profile();
  void load_item_model();
  void load_item_name();
  void save();
  void save_to_impl();
  void add_button_click();
  void delete_button_click();

private:
  std::vector<AgentProfile> _profile_cache;
  std::vector<ModelType> _model_cache;
  TableList* _profile_table;
  TableList* _model_type_table;
  QTabWidget*  _tab_widget;
  QHBoxLayout* _name_layout;
  QLineEdit* _name_value;
};

#endif