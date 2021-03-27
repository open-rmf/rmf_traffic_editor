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

#ifndef TO_STATE_DIALOG__H
#define TO_STATE_DIALOG__H

#include "crowd_sim_dialog.h"
#include "to_state_table.h"

using namespace crowd_sim;

class ToStateDialog final : public CrowdSimDialog
{
public:
  ToStateDialog(
    CrowdSimImplPtr crowd_sim_impl,
    const std::string& dialog_title,
    crowd_sim::Transition& transition);
  void ok_button_click() override;

private:
  std::shared_ptr<ToStateTab> _to_state_tab;
};

#endif