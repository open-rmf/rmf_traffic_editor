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

#include "to_state_dialog.h"

using namespace crowd_sim;

ToStateDialog::ToStateDialog(
  CrowdSimImplPtr crowd_sim_impl,
  const std::string& dialog_title,
  crowd_sim::Transition& transition)
: CrowdSimDialog(crowd_sim_impl, dialog_title)
{
  _to_state_tab = ToStateTab::init_and_make(crowd_sim_impl, transition);
  if (!_to_state_tab)
  {
    throw std::runtime_error(
            "Failed to initialize ToStateTab in ToStateDialog");
  }
  _to_state_tab->update();
  std::string title = "from_state: " + transition.get_from_state();
  setWindowTitle(QString::fromStdString(title) );
  QHBoxLayout* table_box = new QHBoxLayout;
  table_box->addWidget(_to_state_tab.get());
  top_vbox->addLayout(table_box);
  top_vbox->addLayout(bottom_buttons_hbox);
}

//=============================================
void ToStateDialog::ok_button_click()
{
  _to_state_tab->save_to_impl(); //save to current transition
  accept();
}