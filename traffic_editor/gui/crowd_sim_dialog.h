#ifndef CROWD_SIM_DIALOG__H
#define CROWD_SIM_DIALOG__H

#include <memory>

#include <QDialog>
#include <QComboBox>
#include <QtWidgets>

#include <crowd_sim_impl.h>
#include <table_list.h>

class CrowdSimDialog : public QDialog
{
public:
    CrowdSimDialog(CrowdSimImplPtr implPtr);
    ~CrowdSimDialog() {}

    virtual void ok_button_clicked();
    virtual void cancel_button_clicked();

    QPushButton *ok_button, *cancel_button;
    QHBoxLayout* bottom_buttons_hbox;
    QVBoxLayout* top_vbox;
    CrowdSimImplPtr crowd_sim_impl;
};

//====================================================================
class StatesTab;
class StatesDialog : public CrowdSimDialog
{
public:
    StatesDialog(CrowdSimImplPtr implPtr);
    ~StatesDialog() {}

private:
    std::shared_ptr<StatesTab> states_tab;
    void ok_button_clicked() override;
};

class StatesTab : public TableList
{
public:
    StatesTab(CrowdSimImplPtr crowd_sim_impl);
    ~StatesTab() {}

    void update();
    int save();
private:
    CrowdSimImplPtr implPtr;
    int label_size = 0;

    void list_states_in_impl();
    void list_goal_sets_in_combo(QComboBox* comboBox, size_t current_goal_set_id);
    void list_final_states_in_combo(QComboBox* comboBox, bool current_state);
    void list_navmesh_file_in_combo(QComboBox* comboBox, std::string navmesh_filename);
    void add_button_clicked();
};

//=======================================================================================
class GoalSetTab : public TableList
{
public:
    GoalSetTab(CrowdSimImplPtr crowd_sim_impl);
    ~GoalSetTab() {}

    void update();
    int save();

private:
    CrowdSimImplPtr implPtr;
    int label_size = 0;

    void add_button_clicked();
    void list_goal_set_in_impl();
};

class GoalSetDialog : public CrowdSimDialog
{
public:
    GoalSetDialog(CrowdSimImplPtr crowd_sim_impl);
    ~GoalSetDialog() {}

private:
    std::shared_ptr<GoalSetTab> goal_set_tab;
    void ok_button_clicked() override;

};

//==============================================================
class AgentProfileTab : public TableList
{
public :
    AgentProfileTab(CrowdSimImplPtr crowd_sim_impl);
    ~AgentProfileTab() {}

    void update();
    int save();

private:
    CrowdSimImplPtr implPtr;
    int label_size = 0;
    
    void add_button_clicked();
    void list_agent_profile_in_impl();

};

class AgentProfileDialog : public CrowdSimDialog
{
public:
    AgentProfileDialog(CrowdSimImplPtr crowd_sim_impl);
    ~AgentProfileDialog() {}

private:
    std::shared_ptr<AgentProfileTab> agent_profile_tab;
    void ok_button_clicked() override;
};

//==============================================================
class TransitionTab : public TableList
{
public:
    TransitionTab(CrowdSimImplPtr crowd_sim_impl);
    ~TransitionTab() {}

    void update();
    void save();

private:
    CrowdSimImplPtr implPtr;
    int label_size = 0;

    void add_button_clicked();
    void list_transition_in_impl();
    void list_from_states_in_combo(QComboBox*& comboBox, crowd_sim::Transition& transition);
};

class TransitionDialog : public CrowdSimDialog
{
public:
    TransitionDialog(CrowdSimImplPtr crowd_sim_impl);
    ~TransitionDialog() {}

private:
    std::shared_ptr<TransitionTab> transition_tab;
    // void ok_button_clicked() override;

};

class ToStateTab : public TableList 
{
public:
    ToStateTab(crowd_sim::Transition& trasition, CrowdSimImplPtr crowd_sim_impl);
    ~ToStateTab() {}

    void update();
    void save();

private:
    crowd_sim::Transition& current_transition;
    CrowdSimImplPtr implPtr;

    int label_size = 0;
    void list_to_states_in_current_transition();
    void add_button_clicked();

};

class ToStateDialog : public CrowdSimDialog
{
public:
    ToStateDialog(crowd_sim::Transition& transition, CrowdSimImplPtr crowd_sim_impl);
    ~ToStateDialog() {}
private:
    std::shared_ptr<ToStateTab> to_state_tab;

    void ok_button_clicked() override;
};

class ConditionDialog : public CrowdSimDialog 
{
public:
    ConditionDialog(crowd_sim::Transition& transition, CrowdSimImplPtr crowd_sim_impl);
    ~ConditionDialog() {}

private:
    void ok_button_clicked() override;
    void save();
    void update();

    void construct_leaf_condition_widget(
        QWidget*& condition_container, QComboBox*& condition_type, QLineEdit*& condition_value, int condition_index);

    void set_sub_condition_in_root_condition(
        crowd_sim::Condition::TYPE type, double value, int condition_index);

    void initialize_sub_condition(
        crowd_sim::ConditionPtr sub_condition, double value = -1.0);

    crowd_sim::Transition& current_transition;
    QComboBox *root_type;
    QLineEdit *root_value;
    double rootValueD = 0;

    QComboBox *condition1_type, *condition2_type;
    QLineEdit *condition1_value, *condition2_value;
    double condition1ValueD = 0, condition2ValueD = 0;

    QWidget *root_condition_value_container, *condition1_container, *condition2_container;

};

//===========================================================
class AgentGroupTab : public TableList
{
public: 
    AgentGroupTab(CrowdSimImplPtr crowd_sim_impl);
    ~AgentGroupTab() {}

    void update();
    void save();

private:
    CrowdSimImplPtr implPtr;
    int label_size;
    void add_button_clicked();
};

class AgentGroupDialog : public CrowdSimDialog
{
public:
    AgentGroupDialog(CrowdSimImplPtr crowd_sim_impl);
    ~AgentGroupDialog() {}

private:
    std::shared_ptr<AgentGroupTab> agent_group_tab;

};

#endif
