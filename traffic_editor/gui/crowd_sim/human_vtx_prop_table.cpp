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

#include <typeinfo>
#include <iostream>

#include <QString>

#include "human_vtx_prop_table.h"
#include "line_edit.h"

using namespace crowd_sim;

const std::vector<std::string> HumanVtxPropTable::_required_components = {
  "model_type",
  "spawn_number",
  "profile",
  "initial_state",
  "human_goal_set_name"};

//=====================================================================
HumanVtxPropTable::HumanVtxPropTable(const Editor* editor,
  const Project& input_project)
: TableList(2),
  _tab_is_opened(false),
  _actor_models({}),
  _editor(editor),
  _project(input_project)
{
  horizontalHeader()->setVisible(false);
  horizontalHeader()->setSectionResizeMode(
    0,
    QHeaderView::ResizeToContents);
  horizontalHeader()->setSectionResizeMode(
    1,
    QHeaderView::Stretch);
}

//==================================
void HumanVtxPropTable::setActorModels(std::vector<std::string>& am)
{
  _actor_models = am;
}

//==================================
void HumanVtxPropTable::populate(Vertex& vertex, int level)
{
  update(vertex, level);
}

//==================================
void HumanVtxPropTable::close()
{
  if (_tab_is_opened)
  {
    _widget->removeTab(1);
    _tab_is_opened = false;
  }
}

//==============================
void HumanVtxPropTable::update(Vertex& vertex, int level)
{
  if (_project.building.levels.empty())
  {
    close();
    return;
  }

  BuildingLevel lvl = _project.building.levels[level];

  if (lvl.crowd_sim_impl == nullptr)
  {
    printf(
      "Initialize crowd_sim_implementation for project.building level %d\n",
      level + 1);
    _impl = std::make_shared<crowd_sim::CrowdSimImplementation>();
    lvl.crowd_sim_impl = _impl;
  }
  else
  {
    _impl = lvl.crowd_sim_impl;
  }

  int edges_count = lvl.edges.size();
  std::set<int> idxs; // set to keep only unique idx(s)
  for (int i = 0; i < edges_count; i++)
  {
    const auto& edge = lvl.edges.at(i);
    if (edge.type == Edge::Type::HUMAN_LANE)
    {
      idxs.insert(edge.start_idx);
      idxs.insert(edge.end_idx);
    }
  }

  std::vector<std::pair<double, double>> human_lane_vertices;
  QSet<QString> unique_human_goals;
  for (auto& idx:idxs)
  {
    auto& v = lvl.vertices.at(idx);
    human_lane_vertices.push_back({v.x, v.y});
    const auto& it = v.params.find("human_goal_set_name");
    if (it != vertex.params.end())
      unique_human_goals.insert(QString::fromStdString(v.params[
          "human_goal_set_name"].value_string));
  }

  if (std::find(human_lane_vertices.begin(), human_lane_vertices.end(),
    std::pair<double, double>({vertex.x,
      vertex.y})) == human_lane_vertices.end())
  {
    close();
    return;
  }

  QCompleter* completer;
  QList<QString> unique_human_goals_str_list;
  unique_human_goals_str_list = QList<QString>::fromSet(unique_human_goals);
  QStringListModel* unique_human_goals_str_list_model = new QStringListModel(
    unique_human_goals_str_list);
  completer = new QCompleter();
  unique_human_goals_str_list_model->sort(0);
  completer->setModel(unique_human_goals_str_list_model);
  completer->setModelSorting(QCompleter::CaseInsensitivelySortedModel);
  completer->setCompletionMode(QCompleter::UnfilteredPopupCompletion);

  const auto& agent_groups = _impl->get_agent_groups();
  int spawn_number = 0;
  std::string profile = "";
  std::string model_type = "";
  std::string initial_state = "";
  bool found = false;
  for (const auto& group:agent_groups)
  {
    const auto& sp = group.get_spawn_point();
    if (sp.first == dp3(vertex.x) && sp.second == dp3(vertex.y))
    {
      spawn_number = group.get_spawn_number();
      profile = group.get_agent_profile();
      model_type = group.get_internal_agent_model_name();
      initial_state = group.get_initial_state();
      found = true;
      break;
    }
  }

  blockSignals(true);
  const QStringList labels = {"Property", "Value"};
  setHorizontalHeaderLabels(labels);
  setRowCount(5);
  int row_idx = 0;
  for (auto& prop:_required_components)
  {
    QTableWidgetItem* label_item =
      new QTableWidgetItem(QString::fromStdString(prop));
    label_item->setFlags(Qt::NoItemFlags);
    setItem(row_idx, 0, label_item);

    if (prop == "human_goal_set_name") // human_goal_set_name and state_selector (spawn location) have the same location name
    {
      const auto& it = vertex.params.find("human_goal_set_name");
      std::string human_goal_set_name_str =
        (it !=
        vertex.params.end()) ? vertex.params["human_goal_set_name"].value_string
        :
        "";
      LineEdit* goal_line =
        new LineEdit(QString::fromStdString(human_goal_set_name_str));
      goal_line->setCompleter(completer);
      setCellWidget(row_idx, 1, goal_line);
      connect(
        goal_line,
        &QLineEdit::textChanged,
        [this, &vertex](const QString& text)
        {
          if (text.toStdString() == "")
          {
            const auto& it = vertex.params.find("human_goal_set_name");
            vertex.params.erase(it);
          }
          else
            vertex.params["human_goal_set_name"] = Param(text.toStdString());
        }
      );
    }

    if (prop == "initial_state") // human_goal_set_name and state_selector (spawn location) have the same location name
    {
      const auto& states = _impl->get_states();
      auto states_count = states.size();
      QComboBox* state_combo = new QComboBox;
      state_combo->addItem(QString::fromStdString(""));
      for (size_t i = 1; i < states_count; i++)
      {
        auto& current_state = states.at(i);
        auto state_name = current_state.get_name();
        state_combo->addItem(QString::fromStdString(state_name));
        if (initial_state == state_name)
          state_combo->setCurrentIndex(i);
      }
      setCellWidget(row_idx, 1, state_combo);
      connect(
        state_combo,
        &QComboBox::currentTextChanged,
        [this, &vertex](const QString& text)
        {
          auto& agent_groups = _impl->get_agent_groups();
          bool found = false;
          for (auto& group:agent_groups)
          {
            const auto& sp = group.get_spawn_point();
            if (sp.first == dp3(vertex.x) && sp.second == dp3(vertex.y))
            {
              group.set_initial_state(text.toStdString());
              found = true;
              break;
            }
          }
          if (!found) // not found means new group
          {
            AgentGroup agrp(agent_groups.size(), dp3(vertex.x), dp3(vertex.y));
            agrp.set_initial_state(text.toStdString());
            agent_groups.push_back(agrp);
          }
        }
      );
    }

    if (prop == "spawn_number")
    {
      _spawn_number =
        new QLineEdit(((spawn_number >
          0) ? QString::number(spawn_number) : QString::fromStdString("")),
          this);
      setCellWidget(row_idx, 1, _spawn_number);
      connect(
        _spawn_number,
        &QLineEdit::textEdited,
        [this, vertex](const QString& text)
        {
          auto& agent_groups = _impl->get_agent_groups();
          bool found = false;
          for (auto& group:agent_groups)
          {
            const auto& sp = group.get_spawn_point();
            if (sp.first == dp3(vertex.x) && sp.second == dp3(vertex.y))
            {
              group.set_spawn_number(text.toInt());
              found = true;
              break;
            }
          }
          if (!found)
          {
            AgentGroup agrp(agent_groups.size(), dp3(vertex.x), dp3(vertex.y));
            agrp.set_spawn_number(text.toInt());
            agent_groups.push_back(agrp);
          }
        }
      );
    }

    if (prop == "profile")
    {
      const auto& profiles = _impl->get_agent_profiles();
      auto profiles_count = profiles.size();
      QComboBox* profile_combo = new QComboBox;
      profile_combo->addItem(QString::fromStdString(""));
      for (size_t i = 1; i < profiles_count; i++)
      {
        auto& current_profile = profiles.at(i);
        auto& profile_name = current_profile.profile_name;
        profile_combo->addItem(QString::fromStdString(profile_name));
        if (profile == profile_name)
          profile_combo->setCurrentIndex(i);
      }
      setCellWidget(row_idx, 1, profile_combo);
      connect(
        profile_combo,
        &QComboBox::currentTextChanged,
        [this, vertex](const QString& text)
        {
          auto& agent_groups = _impl->get_agent_groups();
          bool found = false;
          for (auto& group:agent_groups)
          {
            const auto& sp = group.get_spawn_point();
            if (sp.first == dp3(vertex.x) && sp.second == dp3(vertex.y))
            {
              group.set_agent_profile(text.toStdString());
              found = true;
              break;
            }
          }
          if (!found)
          {
            AgentGroup agrp(agent_groups.size(), dp3(vertex.x), dp3(vertex.y));
            agrp.set_agent_profile(text.toStdString());
            agent_groups.push_back(agrp);
          }
        }
      );
    }

    if (prop == "model_type")
    {
      auto actors_count = _actor_models.size();
      QComboBox* actor_combo = new QComboBox;
      actor_combo->addItem(QString::fromStdString(""));
      for (size_t i = 0; i < actors_count; i++)
      {
        auto& actor_name = _actor_models.at(i);
        actor_combo->addItem(QString::fromStdString(actor_name));
        if (model_type == actor_name)
          actor_combo->setCurrentIndex(i+1);
      }
      setCellWidget(row_idx, 1, actor_combo);
      connect(
        actor_combo,
        &QComboBox::currentTextChanged,
        [this, vertex](const QString& text)
        {
          auto& agent_groups = _impl->get_agent_groups();
          bool found = false;
          for (auto& group:agent_groups)
          {
            const auto& sp = group.get_spawn_point();
            if (sp.first == dp3(vertex.x) && sp.second == dp3(vertex.y))
            {
              group.set_internal_agent_model_name(text.toStdString());
              found = true;
              break;
            }
          }
          if (!found)
          {
            AgentGroup agrp(agent_groups.size(), dp3(vertex.x), dp3(vertex.y));
            agrp.set_internal_agent_model_name(text.toStdString());
            agent_groups.push_back(agrp);
          }
        }
      );
    }
    row_idx++;
  }

  if (!_tab_is_opened)
  {
    _widget->addTab(this, "Human");
    _tab_is_opened = true;
    setStyleSheet(_tabStyle);
  }

  blockSignals(false);
}