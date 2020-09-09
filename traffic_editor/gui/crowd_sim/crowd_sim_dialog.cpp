#include "crowd_sim_dialog.h"

using namespace crowd_sim;

//===========================================================
CrowdSimDialog::CrowdSimDialog()
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
}

//=======================================================
GoalSetDialog::GoalSetDialog(CrowdSimImplPtr crowd_sim_impl)
{
  _goal_set_tab = GoalSetTab::init_and_make(crowd_sim_impl);
  if (!_goal_set_tab)
  {
    throw std::runtime_error("Failed to initialize GoalSetTab in GoalSetDialog");
  }
  _goal_set_tab->update();

  setWindowTitle("Goal Sets");
  QHBoxLayout* table_box = new QHBoxLayout;
  table_box->addWidget(_goal_set_tab.get());
  top_vbox->addLayout(table_box);
  top_vbox->addLayout(bottom_buttons_hbox);
}

//=========================================================
StateDialog::StateDialog(CrowdSimImplPtr crowd_sim_impl)
{
  _state_tab = StatesTab::init_and_make(crowd_sim_impl);
  if (!_state_tab)
  {
    throw std::runtime_error("Failed to initialize StateTab in StateDialog");
  }
  _state_tab->update();

  setWindowTitle("States");
  QHBoxLayout* table_box = new QHBoxLayout;
  table_box->addWidget(_state_tab.get());
  top_vbox->addLayout(table_box);
  top_vbox->addLayout(bottom_buttons_hbox);
}

//=========================================================
TransitionDialog::TransitionDialog(CrowdSimImplPtr crowd_sim_impl)
{
  _transition_tab = TransitionTab::init_and_make(crowd_sim_impl);
  if (!_transition_tab)
  {
    throw std::runtime_error(
            "Failed to initialize TransitionTab in TransitionDialog");
  }
  _transition_tab->update();

  setWindowTitle("Transitions");
  QHBoxLayout* table_box = new QHBoxLayout;
  table_box->addWidget(_transition_tab.get());
  top_vbox->addLayout(table_box);
  top_vbox->addLayout(bottom_buttons_hbox);
}

//=========================================================
ToStateDialog::ToStateDialog(CrowdSimImplPtr crowd_sim_impl,
  crowd_sim::Transition& transition)
{
  _to_state_tab = ToStateTab::init_and_make(crowd_sim_impl, transition);
  if (!_to_state_tab)
  {
    throw std::runtime_error("Failed to initialize ToStateTab in ToStateDialog");
  }
  _to_state_tab->update();
  std::string title = "from_state: " + transition.get_from_state();
  setWindowTitle(QString::fromStdString(title) );
  QHBoxLayout* table_box = new QHBoxLayout;
  table_box->addWidget(_to_state_tab.get());
  top_vbox->addLayout(table_box);
  top_vbox->addLayout(bottom_buttons_hbox);
}

//=========================================================
AgentProfileDialog::AgentProfileDialog(CrowdSimImplPtr crowd_sim_impl)
{
  _agent_profile_tab = AgentProfileTab::init_and_make(crowd_sim_impl);
  if (!_agent_profile_tab)
  {
    throw std::runtime_error(
            "Failed to initialize AgentProfileTab in AgentProfileDialog");
  }
  _agent_profile_tab->update();
  setWindowTitle("AgentProfile");
  QHBoxLayout* table_box = new QHBoxLayout;
  table_box->addWidget(_agent_profile_tab.get());
  top_vbox->addLayout(table_box);
  top_vbox->addLayout(bottom_buttons_hbox);
}

//=========================================================
AgentGroupDialog::AgentGroupDialog(CrowdSimImplPtr crowd_sim_impl)
{
  _agent_group_tab = AgentGroupTab::init_and_make(crowd_sim_impl);
  if (!_agent_group_tab)
  {
    throw std::runtime_error(
            "Failed to initialize AgentGroupTab in AgentGroupDialog");
  }
  _agent_group_tab->update();
  setWindowTitle("AgentGroup");
  QHBoxLayout* table_box = new QHBoxLayout;
  table_box->addWidget(_agent_group_tab.get());
  top_vbox->addLayout(table_box);
  top_vbox->addLayout(bottom_buttons_hbox);
}

//=========================================================
ModelTypeDialog::ModelTypeDialog(CrowdSimImplPtr crowd_sim_impl)
{
  _model_type_tab = ModelTypeTab::init_and_make(crowd_sim_impl);
  if (!_model_type_tab)
  {
    throw std::runtime_error(
            "Failed to initialize ModelTypeTab in ModelTypeDialog");
  }
  _model_type_tab->update();
  setWindowTitle("ModelType");
  QHBoxLayout* table_box = new QHBoxLayout;
  table_box->addWidget(_model_type_tab.get());
  top_vbox->addLayout(table_box);
  top_vbox->addLayout(bottom_buttons_hbox);
}