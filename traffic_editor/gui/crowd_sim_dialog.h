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

    void list_states_in_impl();
    void list_goal_sets_in_combo(QComboBox* comboBox, size_t current_goal_set_id);
    void list_final_states_in_combo(QComboBox* comboBox, bool current_state);
    void add_button_clicked();
};


class GoalSetTab : public TableList
{
public:
    GoalSetTab(CrowdSimImplPtr crowd_sim_impl);
    ~GoalSetTab() {}

    void update();
    int save();

private:
    CrowdSimImplPtr implPtr;

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

#endif