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

#ifndef CROWD_SIM_CROWDSIM_LAYOUT_BASE__H
#define CROWD_SIM_CROWDSIM_LAYOUT_BASE__H

#include <memory>

#include <traffic_editor/crowd_sim/crowd_sim_impl.h>

#include <QtWidgets>

using namespace crowd_sim;

class CrowdSimLayoutBase : public QHBoxLayout
{
public:
  CrowdSimLayoutBase(CrowdSimImplPtr impl);
  virtual ~CrowdSimLayoutBase() {}

  CrowdSimImplPtr get_impl() const { return _crowd_sim_impl; }

  virtual int get_cache_size() const = 0;
  virtual void list_item_in_cache() = 0;
  virtual void initialise_item_detail() = 0;
  virtual void update_item_detail() = 0;
  virtual void save() = 0;
  virtual void save_to_impl() = 0;
  virtual void add_button_click() = 0;
  virtual void delete_button_click() = 0;
  virtual void initialise();
  virtual void update();

private:
  std::vector<QLayoutItem*> _layout_items;
  QTabWidget* _right_tab_widget;
  CrowdSimImplPtr _crowd_sim_impl;

protected:
  uint _current_index;
  bool _to_delete;
  QListWidget* _list_widget;
  QVBoxLayout* _right_panel;
};

using CrowdSimLayoutPtr = std::shared_ptr<CrowdSimLayoutBase>;

#endif