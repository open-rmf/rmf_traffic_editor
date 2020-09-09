#ifndef CROWD_SIM_DIALOG__H
#define CROWD_SIM_DIALOG__H

#include <memory>

#include <QDialog>
#include <QComboBox>
#include <QtWidgets>

#include <traffic_editor/crowd_sim/crowd_sim_impl.h>

#include "goal_set_table.h"
#include "state_table.h"
#include "transition_table.h"
#include "to_state_table.h"
#include "agent_profile_table.h"
#include "agent_group_table.h"
#include "model_type_table.h"

using namespace crowd_sim;

//======================================================
class CrowdSimDialog : public QDialog
{
public:
    CrowdSimDialog();
    virtual ~CrowdSimDialog() {}

    QPushButton *ok_button, *cancel_button;
    QHBoxLayout *bottom_buttons_hbox;
    QVBoxLayout *top_vbox;

    virtual void ok_button_click() { accept(); }
    void cancel_button_click() { reject(); }
};

//=======================================================
class GoalSetDialog final : public CrowdSimDialog
{
public: 
    GoalSetDialog(CrowdSimImplPtr crowd_sim_impl);
    void ok_button_click() override
    { 
        _goal_set_tab->save_to_impl();
        accept();
    }

private:
    std::shared_ptr<GoalSetTab> _goal_set_tab;
};

//=======================================================
class StateDialog final : public CrowdSimDialog
{
public:
    StateDialog(CrowdSimImplPtr crowd_sim_impl);
    void ok_button_click() override
    {
        _state_tab->save_to_impl();
        accept();
    }
private:
    std::shared_ptr<StatesTab> _state_tab;
};

//=========================================================
class TransitionDialog final : public CrowdSimDialog
{
public:
    TransitionDialog(CrowdSimImplPtr crowd_sim_impl);
    void ok_button_click() override
    {
        _transition_tab->save_to_impl();
        accept();
    }
private:
    std::shared_ptr<TransitionTab> _transition_tab;
};

//=========================================================
class ToStateDialog final : public CrowdSimDialog
{
public:
    ToStateDialog(CrowdSimImplPtr crowd_sim_impl, crowd_sim::Transition& transition);
    void ok_button_click() override
    {
        _to_state_tab->save_to_impl(); //save to current transition
        accept();
    }
private:
    std::shared_ptr<ToStateTab> _to_state_tab;
};

//=========================================================
class AgentProfileDialog final : public CrowdSimDialog
{
public:
    AgentProfileDialog(CrowdSimImplPtr crowd_sim_impl);
    void ok_button_click() override
    {
        _agent_profile_tab->save_to_impl();
        accept();
    }
private:
    std::shared_ptr<AgentProfileTab> _agent_profile_tab;
};

//=========================================================
class AgentGroupDialog final : public CrowdSimDialog
{
public:
    AgentGroupDialog(CrowdSimImplPtr crowd_sim_impl);
    void ok_button_click() override
    {
        _agent_group_tab->save_to_impl();
        accept();
    }
private:
    std::shared_ptr<AgentGroupTab> _agent_group_tab;
};

//=========================================================
class ModelTypeDialog final : public CrowdSimDialog
{
public:
    ModelTypeDialog(CrowdSimImplPtr crowd_sim_impl);
    void ok_button_click() override
    {
        _model_type_tab->save_to_impl();
        accept();
    }
private:
    std::shared_ptr<ModelTypeTab> _model_type_tab;
};


#endif