#ifndef CROWD_SIM_DIALOG__H
#define CROWD_SIM_DIALOG__H

#include <memory>

#include <QDialog>

#include <crowd_sim_impl.h>
#include <table_list.h>

// class CrowdSimDialog : public QDialog
// {
// public:
//     CrowdSimDialog(CrowdSimImplPtr implPtr);
//     ~CrowdSimDialog() {}

//     QPushButton *ok_button, *cancel_button;
//     CrowdSimImplPtr crowd_sim_impl;

// };

class StatesTab;
class StatesDialog : public QDialog
{
public:
    StatesDialog(CrowdSimImplPtr implPtr);
    ~StatesDialog() {}

    // void update();

private:
    CrowdSimImplPtr crowd_sim_impl;

    std::shared_ptr<StatesTab> states_tab = std::make_shared<StatesTab>();

// public slots:
//     void ok_button_clicked();
};

class StatesTab : public TableList
{
public:
    StatesTab();
    ~StatesTab() {}

    // void update(CrowdSimImplPtr implPtr);
};




#endif