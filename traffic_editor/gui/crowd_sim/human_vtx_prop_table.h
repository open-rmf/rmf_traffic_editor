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

#ifndef HUMAN_VTX_PROP_TABLE__H
#define HUMAN_VTX_PROP_TABLE__H

#include <vector>
#include <string>
#include <set>

#include <QTableWidget>
#include <QtWidgets>

#include "table_list.h"
#include "editor.h"
#include "building.h"
#include <traffic_editor/crowd_sim/crowd_sim_impl.h>

class Editor;
using namespace crowd_sim;

class HumanVtxPropTable : public TableList
{
  Q_OBJECT;

public:
  HumanVtxPropTable(const Editor* editor, const Building& building);
  ~HumanVtxPropTable() {}

  void setActorModels(std::vector<std::string>& am);
  void populate(Vertex& vertex, int level);
  void close();
  void update(Vertex& vertex, int level);
  void storeWidget(QTabWidget* widget) {_widget = widget;}
  void storeTabStyle(QString tabStyle) {_tabStyle = tabStyle;}

  static const std::vector<std::string> _required_components;

private:
  bool _tab_is_opened;
  const Building& _building;
  CrowdSimImplPtr _impl;
  QTabWidget* _widget;
  std::vector<std::string> _actor_models;
  QLineEdit* _spawn_number;
  QString _tabStyle;
  const Editor* _editor;
};

#endif
