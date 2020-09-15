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

#ifndef CROWD_SIM_CONDITION_DIALOG__H
#define CROWD_SIM_CONDITION_DIALOG__H

#include <QComboBox>
#include <QWidget>
#include <QLineEdit>

#include <traffic_editor/crowd_sim/crowd_sim_impl.h>
#include <traffic_editor/crowd_sim/condition.h>

#include "crowd_sim_dialog.h"

using namespace crowd_sim;

class ConditionDialog : public CrowdSimDialog
{
public:
  ConditionDialog(
    CrowdSimImplPtr crowd_sim_impl,
    const std::string& dialog_title,
    Transition& transition);
  ~ConditionDialog() {}

  void save();
  void update();
  void ok_button_click() override;

private:
  crowd_sim::Transition& _current_transition;

  QComboBox* _root_type;
  QLineEdit* _root_value;
  double _rootValueD = 0;

  QComboBox* _condition1_type, * _condition2_type;
  QLineEdit* _condition1_value, * _condition2_value;
  double _condition1ValueD = 0, _condition2ValueD = 0;

  QWidget* _root_condition_value_container, * _condition1_container,
    * _condition2_container;

  void _construct_root_condition_type(
    QComboBox* root_type,
    crowd_sim::Condition::TYPE current_type);

  void _construct_leaf_condition_widget(
    QWidget* condition_container,
    QComboBox* condition_type,
    QLineEdit* condition_value,
    int condition_index);

  void _set_sub_condition_in_root_condition(
    crowd_sim::Condition::TYPE type,
    double value,
    int condition_index);

  void _root_is_leaf_condition();
  void _root_is_bool2_condition();
  void _root_is_not_condition();
  void _root_is_invalid();
};

#endif