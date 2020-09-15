#include "crowd_sim_dialog.h"
#include "state_table.h"
#include "goal_set_table.h"
#include "transition_table.h"
#include "to_state_table.h"
#include "agent_profile_table.h"
#include "agent_group_table.h"
#include "model_type_table.h"

using namespace crowd_sim;

//===========================================================
CrowdSimDialog::CrowdSimDialog(
  CrowdSimImplPtr crowd_sim_impl,
  const std::string& dialog_title)
{
  ok_button = new QPushButton("OK", this);
  cancel_button = new QPushButton("Cancel", this);
  bottom_buttons_hbox = new QHBoxLayout;
  bottom_buttons_hbox->addWidget(cancel_button);
  bottom_buttons_hbox->addWidget(ok_button);
  connect(
    ok_button,
    &QAbstractButton::clicked,
    [this]()
    {
      ok_button_click();
    }
  );
  connect(
    cancel_button,
    &QAbstractButton::clicked,
    [this]()
    {
      cancel_button_click();
    }
  );
  top_vbox = new QVBoxLayout(this);
   
  if ("States" == dialog_title)
  {
    _table_ptr = StatesTab::init_and_make(crowd_sim_impl);
  }
  else if ("GoalSets" == dialog_title)
  {
    _table_ptr = GoalSetTab::init_and_make(crowd_sim_impl);
  }
  else if ("AgentProfiles" == dialog_title)
  {
    _table_ptr = AgentProfileTab::init_and_make(crowd_sim_impl);
  }
  else if ("Transitions" == dialog_title)
  {
    _table_ptr = TransitionTab::init_and_make(crowd_sim_impl);
  }
  else if ("AgentGroups" == dialog_title)
  {
    _table_ptr = AgentProfileTab::init_and_make(crowd_sim_impl);
  }
  else if ("ModelTypes" == dialog_title)
  {
    _table_ptr = ModelTypeTab::init_and_make(crowd_sim_impl);
  }
  else {
    // stop constructing table with other type of dialog
    _table_ptr = nullptr;
    return;
  }

  if (!_table_ptr){
    throw std::runtime_error(
            "Failed to initialize table in Dialog " + dialog_title);
  }
  _table_ptr->update();
  QHBoxLayout* table_box = new QHBoxLayout;
  table_box->addWidget(_table_ptr.get());
  top_vbox->addLayout(table_box);
  top_vbox->addLayout(bottom_buttons_hbox);
  setWindowTitle(QString::fromStdString(dialog_title));
}

//==============================================
void CrowdSimDialog::ok_button_click()
{
  if (_table_ptr)
    _table_ptr->save_to_impl();
  accept();
}

//==============================================
void CrowdSimDialog::cancel_button_click()
{
  reject();
}
