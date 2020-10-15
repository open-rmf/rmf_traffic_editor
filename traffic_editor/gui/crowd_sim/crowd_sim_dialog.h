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

#ifndef CROWD_SIM_DIALOG__H
#define CROWD_SIM_DIALOG__H

#include <memory>

#include <QDialog>
#include <QComboBox>
#include <QtWidgets>

#include <traffic_editor/crowd_sim/crowd_sim_impl.h>

#include "crowd_sim_table_base.h"

using namespace crowd_sim;

class CrowdSimDialog : public QDialog
{
public:
  CrowdSimDialog(
    CrowdSimImplPtr crowd_sim_impl,
    const std::string& dialog_title);
  virtual ~CrowdSimDialog() {}

  QPushButton* ok_button, * cancel_button;
  QHBoxLayout* bottom_buttons_hbox;
  QVBoxLayout* top_vbox;

  virtual void ok_button_click();
  void cancel_button_click();

private:
  CrowdSimTablePtr _table_ptr;
};

#endif