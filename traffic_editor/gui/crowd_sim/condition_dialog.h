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
    ConditionDialog(Transition& transition);
    ~ConditionDialog() {}

    void save();
    void update();
    void ok_button_click() override
    {
        save();
        accept();
    }

private:
    CrowdSimImplPtr _impl;
    crowd_sim::Transition& _current_transition;

    QComboBox *_root_type;
    QLineEdit *_root_value;
    double _rootValueD = 0;

    QComboBox *_condition1_type, *_condition2_type;
    QLineEdit *_condition1_value, *_condition2_value;
    double _condition1ValueD = 0, _condition2ValueD = 0;

    QWidget *_root_condition_value_container, *_condition1_container, *_condition2_container;

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