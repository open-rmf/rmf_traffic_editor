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

#ifndef CROWD_SIM_CROWDSIM_TABLE_BASE__H
#define CROWD_SIM_CROWDSIM_TABLE_BASE__H

#include <memory>

#include <traffic_editor/crowd_sim/crowd_sim_impl.h>

#include "table_list.h"

using namespace crowd_sim;

class CrowdSimTableBase : public TableList
{
public:
  CrowdSimTableBase(CrowdSimImplPtr impl, const QStringList& labels);
  virtual ~CrowdSimTableBase() {}

  CrowdSimImplPtr get_impl() const { return _crowd_sim_impl; }
  size_t get_label_size() const { return _label_size; }
  void set_label_size(size_t label_size) { _label_size = label_size; }

  virtual int get_cache_size() const = 0;
  virtual void list_item_in_cache() = 0;
  virtual void save() = 0;
  virtual void save_to_impl() = 0;
  virtual void add_button_click() = 0;
  virtual void delete_button_click(size_t row_num) = 0;

  virtual void update();

private:
  CrowdSimImplPtr _crowd_sim_impl;
  size_t _label_size;
};

using CrowdSimTablePtr = std::shared_ptr<CrowdSimTableBase>;

#endif