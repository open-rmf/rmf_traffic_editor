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