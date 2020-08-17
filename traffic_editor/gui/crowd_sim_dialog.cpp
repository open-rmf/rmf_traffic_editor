#include <crowd_sim_dialog.h>
#include <multi_select_combo_box.h>

#include <iostream>
#include <sstream>
#include <string>

CrowdSimDialog::CrowdSimDialog(CrowdSimImplPtr implPtr) 
: crowd_sim_impl(implPtr)
{
    ok_button = new QPushButton("OK", this);
    cancel_button = new QPushButton("Cancel", this);
    bottom_buttons_hbox = new QHBoxLayout;
    bottom_buttons_hbox->addWidget(cancel_button);
    bottom_buttons_hbox->addWidget(ok_button);
    connect(
        ok_button,
        &QAbstractButton::clicked,
        [this](){
            this->ok_button_clicked();
        }
    );
    connect(
        cancel_button,
        &QAbstractButton::clicked,
        [this](){
            this->cancel_button_clicked();
        }
    );
    top_vbox = new QVBoxLayout(this);
}

void CrowdSimDialog::ok_button_clicked() {
    accept();
}

void CrowdSimDialog::cancel_button_clicked() {
    reject();
}

//==========================================================================
StatesDialog::StatesDialog(CrowdSimImplPtr implPtr) 
    : CrowdSimDialog(implPtr) 
{
    states_tab = std::make_shared<StatesTab>(crowd_sim_impl);
    states_tab->update();

    setWindowTitle("States");

    QHBoxLayout* table_box = new QHBoxLayout;
    table_box->addWidget(states_tab.get());

    top_vbox->addLayout(table_box);
    top_vbox->addLayout(bottom_buttons_hbox);

}

void StatesDialog::ok_button_clicked() {
    states_tab->save();
    accept();
}

StatesTab::StatesTab(CrowdSimImplPtr crowd_sim_impl) 
    : TableList(5), implPtr(crowd_sim_impl)
{
    const QStringList labels =
        { "Name", "Is Final", "Navmesh File Name", "Goal Set Id", ""};
    setHorizontalHeaderLabels(labels);
    setMinimumSize(800, 400);
}

void StatesTab::update()
{
    auto states_number = implPtr->states.size();    
    setRowCount(1 + states_number);
    clearContents();

    list_states_in_impl();

    QPushButton* add_button = new QPushButton("Add...", this);
    for (auto i = 0; i < 4; i++) {
        setItem(states_number, i, new QTableWidgetItem(QString::fromStdString("")));
    }
    setCellWidget(states_number, 4, add_button);
    connect(
        add_button,
        &QAbstractButton::clicked,
        [this](){
            add_button_clicked();
            update();
        }
    );
}

void StatesTab::list_states_in_impl() {

    auto states_number = implPtr->states.size();

    //row 0 for external_state
    auto& current_state = implPtr->states.at(0);
    setItem(0, 0, new QTableWidgetItem(QString::fromStdString(current_state.getName() )));
    setItem(0, 1, new QTableWidgetItem(QString::number(current_state.getFinalState()? 1:0 )));
    setItem(0, 2, new QTableWidgetItem(QString::fromStdString("")));
    setItem(0, 3, new QTableWidgetItem(QString::fromStdString("")));
    setItem(0, 4, new QTableWidgetItem(QString::fromStdString("")));

    for (size_t i = 1; i < states_number; i++) {
        auto& current_state = implPtr->states.at(i);
        setItem(i, 0, new QTableWidgetItem(QString::fromStdString(current_state.getName()) ) );

        QComboBox* final_state_combo = new QComboBox;
        list_final_states_in_combo(final_state_combo, current_state.getFinalState());
        setCellWidget(i, 1, final_state_combo );
        connect(
            final_state_combo,
            &QComboBox::currentTextChanged,
            [&](const QString& text){
                current_state.setFinalState(text.toStdString() == "1");
            }
        );

        QComboBox* navmesh_list_combo = new QComboBox;
        list_navmesh_file_in_combo(navmesh_list_combo, current_state.getNavmeshFileName());
        setCellWidget(i, 2, navmesh_list_combo);
        connect(
            navmesh_list_combo,
            &QComboBox::currentTextChanged,
            [&](const QString& text){
                current_state.setNavmeshFile(text.toStdString());
            }
        );
        // setItem(i, 2, new QTableWidgetItem(QString::fromStdString(current_state.getNavmeshFileName() )));

        QComboBox* goal_set_combo = new QComboBox;
        list_goal_sets_in_combo(goal_set_combo, current_state.getGoalSetId());
        setCellWidget(i, 3, goal_set_combo);
        connect(
            goal_set_combo,
            &QComboBox::currentTextChanged,
            [&](const QString& text){
                size_t goal_set_id;
                std::istringstream(text.toStdString()) >> goal_set_id;
                current_state.setGoalSetId(goal_set_id);
            }
        );
        
        QPushButton* delete_button = new QPushButton("Delete", this);
        setCellWidget(i, 4, delete_button);
        connect(
            delete_button,
            &QAbstractButton::clicked,
            [&, i](){
                implPtr->states.erase(implPtr->states.begin() + i);
                update();
            }
        );
    }

}

void StatesTab::list_goal_sets_in_combo(QComboBox* comboBox, size_t current_goal_set_id) {
    for (auto goal_set : implPtr->goal_sets) {
        comboBox->addItem(QString::number(static_cast<int>(goal_set.getGoalSetId())));
    }
    auto index = comboBox->findText(QString::number(static_cast<int>(current_goal_set_id) ));
    if (index >= 0) {
        comboBox->setCurrentIndex(index);
    }
}

void StatesTab::list_final_states_in_combo(QComboBox* comboBox, bool current_state) {
    comboBox->addItem("1");
    comboBox->addItem("0");
    if (!current_state) {
        comboBox->setCurrentIndex(1);
    } else {
        comboBox->setCurrentIndex(0);
    }
}

void StatesTab::list_navmesh_file_in_combo(QComboBox* comboBox, std::string navmesh_filename) {
    auto navmesh_list = implPtr->getNavmeshFileName();
    for (auto i = 0; i < navmesh_list.size(); i++) {
        comboBox->addItem(QString::fromStdString(navmesh_list[i]));
        if (navmesh_list[i] == navmesh_filename) {
            comboBox->setCurrentIndex(i);
        }
    }
}

void StatesTab::add_button_clicked() {
    save();
    implPtr->states.emplace_back("new state");
}

int StatesTab::save() {
    auto rows_count = rowCount();
    //external_state is not allowed to be changed
    for (auto i = 1; i < rows_count - 1; i++) {
        auto& current_state = implPtr->states.at(i);
        //state_name
        auto name_item = item(i, 0)->text().toStdString();
        current_state.setName(name_item);
        //is_final
        auto final_item = static_cast<QComboBox*>(cellWidget(i, 1))->currentText().toStdString();
        if (final_item == "1") {
            current_state.setFinalState(true);
            continue;
        }
        else
            current_state.setFinalState(false);
        //navmeshfile_name
        auto navmeshfile_name = static_cast<QComboBox*>(cellWidget(i, 2))->currentText().toStdString();
        current_state.setNavmeshFile(navmeshfile_name);
        //goal_set_id
        bool OK_status;
        auto goal_set_id = static_cast<QComboBox*>(cellWidget(i, 3))->currentText().toInt(&OK_status);
        if(!OK_status && goal_set_id >= 0) {
            std::cout << "Invalid goal set id provided." << std::endl;
            continue;
        }
        current_state.setGoalSetId(static_cast<size_t>(goal_set_id));
        
    }
    return rows_count;
}

//==============================================================================

GoalSetTab::GoalSetTab(CrowdSimImplPtr crowd_sim_impl) 
    : TableList(3), implPtr(crowd_sim_impl) 
{
    const QStringList labels =
        { "Id", "Goal Area", ""};

    setHorizontalHeaderLabels(labels);
    horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    setMinimumSize(800, 400);
}

void GoalSetTab::update() {
    blockSignals(true);
    auto goal_set_number = implPtr->goal_sets.size();
    setRowCount(1 + goal_set_number);
    clearContents();
    list_goal_set_in_impl();

    QPushButton* add_button = new QPushButton("Add...", this);
    for (auto i = 0; i < 2; i++) {
        setItem(goal_set_number, i, new QTableWidgetItem(QString::fromStdString("")));
    }
    setCellWidget(goal_set_number, 2, add_button);
    connect(
        add_button,
        &QAbstractButton::clicked,
        [&](){
            add_button_clicked();
            update();
        }
    );
    blockSignals(false);

}

void GoalSetTab::list_goal_set_in_impl() {
    blockSignals(true);
    auto goal_set_number = implPtr->goal_sets.size();
    for (auto i = 0; i < goal_set_number; i++) {
        auto& goal_set = implPtr->goal_sets[i];
        setItem(i, 0, new QTableWidgetItem(QString::number(static_cast<int>(goal_set.getGoalSetId()))));

        MultiSelectComboBox* multi_combo_box = new MultiSelectComboBox(implPtr->getGoalAreas());
        multi_combo_box->showCheckedItem(goal_set.getGoalAreas());
        setCellWidget(i, 1, multi_combo_box);
    }
    blockSignals(false);
}

int GoalSetTab::save() {
    auto row_count = rowCount();
    implPtr->goal_sets.clear();

    for (auto i = 0; i < row_count - 1; i++) {
        
        QTableWidgetItem* pItem_setid = item(i, 0);
        bool OK_status;
        auto set_id = pItem_setid->text().toInt(&OK_status);
        if(!OK_status) {
            std::cout << "Invalid goal set id in row " << i << " with [" << pItem_setid->text().toStdString() << "]";
            return -1; // should try exception?
        }

        auto pItem_areas = static_cast<MultiSelectComboBox*>(cellWidget(i, 1));
        if (pItem_areas->getCheckResult().empty()) { //if no goal area selected, don't save the goal set
            continue;
        }
        implPtr->goal_sets.emplace_back(static_cast<size_t>(set_id));
        auto& goal_set_iterator = implPtr->goal_sets.back();
        for(auto item : pItem_areas->getCheckResult() ) {
            goal_set_iterator.addGoalArea(item);
        }

    }
    return row_count;
}

void GoalSetTab::add_button_clicked() {
    auto row_count = save();
    size_t new_goal_set_id = 0;
    if(implPtr->goal_sets.size() != 0) {
        new_goal_set_id = implPtr->goal_sets.back().getGoalSetId() + 1;
    }
    implPtr->goal_sets.emplace_back(new_goal_set_id);
}


GoalSetDialog::GoalSetDialog(CrowdSimImplPtr crowd_sim_impl)
    : CrowdSimDialog(crowd_sim_impl)
{
    goal_set_tab = std::make_shared<GoalSetTab>(crowd_sim_impl);
    goal_set_tab->update();

    setWindowTitle("Goal Sets");

    QHBoxLayout* table_box = new QHBoxLayout;
    table_box->addWidget(goal_set_tab.get());

    top_vbox->addLayout(table_box);
    top_vbox->addLayout(bottom_buttons_hbox);
}

void GoalSetDialog::ok_button_clicked() {
    goal_set_tab->save();
    accept();
}

//===================================================================
AgentProfileTab::AgentProfileTab(CrowdSimImplPtr crowd_sim_impl) 
    : TableList(13), implPtr(crowd_sim_impl)
{
    const QStringList labels =
        { "Name", "class", "max_accel", "max_angle_vel", "max_neighbors",
        "max_speed", "neighbor_dist", "obstacle_set", "pref_speed", "r",
        "ORCA_tau", "ORCA_tauObst", ""};
    
    setHorizontalHeaderLabels(labels);
    setMinimumSize(1600, 400);

}

void AgentProfileTab::add_button_clicked() {
    save();
    implPtr->agent_profiles.emplace_back("new profile");
}

void AgentProfileTab::list_agent_profile_in_impl() {
    blockSignals(true);
    auto profile_count = implPtr->agent_profiles.size();
    
    for (auto i = 0; i < profile_count; i++) {
        auto& current_profile = implPtr->agent_profiles[i];
        setItem(i, 0, new QTableWidgetItem(QString::fromStdString(current_profile.profile_name)));
        setItem(i, 1, new QTableWidgetItem(QString::number(static_cast<uint>(current_profile.profile_class))));
        setItem(i, 2, new QTableWidgetItem(QString::number(current_profile.max_accel)));
        setItem(i, 3, new QTableWidgetItem(QString::number(current_profile.max_angle_vel)));
        setItem(i, 4, new QTableWidgetItem(QString::number(static_cast<uint>(current_profile.max_neighbors))));
        setItem(i, 5, new QTableWidgetItem(QString::number(current_profile.max_speed)));
        setItem(i, 6, new QTableWidgetItem(QString::number(current_profile.neighbor_dist)));
        setItem(i, 7, new QTableWidgetItem(QString::number(static_cast<uint>(current_profile.obstacle_set))));
        setItem(i, 8, new QTableWidgetItem(QString::number(current_profile.pref_speed)));
        setItem(i, 9, new QTableWidgetItem(QString::number(current_profile.r)));
        setItem(i, 10, new QTableWidgetItem(QString::number(current_profile.ORCA_tau)));
        setItem(i, 11, new QTableWidgetItem(QString::number(current_profile.ORCA_tauObst)));
        
        // not permit to delete the external agent profile
        if (i == 0) continue;
        QPushButton* delete_button = new QPushButton("Delete", this);
        setCellWidget(i, 12, delete_button);
        connect(
            delete_button,
            &QAbstractButton::clicked,
            [&, i](){
                implPtr->agent_profiles.erase(implPtr->agent_profiles.begin() + i);
                update();
            }
        );
    }
    blockSignals(false);
}

void AgentProfileTab::update() {
    blockSignals(true);
    auto profiles_number = implPtr->agent_profiles.size();
    setRowCount(1 + profiles_number);
    clearContents();
    list_agent_profile_in_impl();

    QPushButton* add_button = new QPushButton("Add...", this);
    for (auto i = 0; i < 11; i++) {
        setItem(profiles_number, i, new QTableWidgetItem(QString::fromStdString("")));
    }
    setCellWidget(profiles_number, 12, add_button);
    connect(
        add_button,
        &QAbstractButton::clicked,
        [&](){
            add_button_clicked();
            update();
        }
    );
    blockSignals(false);

}

int AgentProfileTab::save() {
    auto row_count = rowCount();
    //external_agent profile is not allowed to be changed
    for (auto i = 1; i < row_count - 1; i++) {
        QTableWidgetItem* pItem = item(i, 0);
        auto profile_name = pItem->text();
        auto& current_profile = implPtr->agent_profiles.at(i);
        
        bool OK_status;
        pItem = item(i, 1); //profile_class
        auto profile_class = pItem->text().toInt(&OK_status);
        if(!OK_status) {
            std::cout << "Error in saving profile_class for Agent Profile: [" 
                << profile_name.toStdString() << "]" << std::endl;
            return -1;
        }
        current_profile.profile_class = static_cast<size_t>(profile_class);

        pItem = item(i, 2); //max_accel
        auto max_accel = pItem->text().toDouble(&OK_status);
        if(!OK_status) {
            std::cout << "Error in saving max_accel for Agent Profile: [" 
                << profile_name.toStdString() << "]" << std::endl;
            return -1;
        }
        current_profile.max_accel = static_cast<double>(max_accel);

        pItem = item(i, 3); //max_angle_vel
        auto max_angle_vel = pItem->text().toDouble(&OK_status);
        if(!OK_status) {
            std::cout << "Error in saving max_angle_vel for Agent Profile: [" 
                << profile_name.toStdString() << "]" << std::endl;
            return -1;
        }
        current_profile.max_angle_vel = static_cast<double>(max_angle_vel);

        pItem = item(i, 4); //max_neighbors
        auto max_neighbors = pItem->text().toInt(&OK_status);
        if(!OK_status) {
            std::cout << "Error in saving max_neighbors for Agent Profile: [" 
                << profile_name.toStdString() << "]" << std::endl;
            return -1;
        }
        current_profile.max_neighbors = static_cast<size_t>(max_neighbors);

        pItem = item(i, 5); //max_speed
        auto max_speed = pItem->text().toDouble(&OK_status);
        if(!OK_status) {
            std::cout << "Error in saving max_speed for Agent Profile: [" 
                << profile_name.toStdString() << "]" << std::endl;
            return -1;
        }
        current_profile.max_speed = static_cast<double>(max_speed);

        pItem = item(i, 6); //neighbor_dist
        auto neighbor_dist = pItem->text().toDouble(&OK_status);
        if(!OK_status) {
            std::cout << "Error in saving neighbor dist for Agent Profile: [" 
                << profile_name.toStdString() << "]" << std::endl;
            return -1;            
        }
        current_profile.neighbor_dist = static_cast<double>(neighbor_dist);

        pItem = item(i, 7); //obstacle_set
        auto obstacle_set = pItem->text().toInt(&OK_status);
        if(!OK_status) {
            std::cout << "Error in saving obstacle_set for Agent Profile: [" 
                << profile_name.toStdString() << "]" << std::endl;
            return -1;            
        }       
        current_profile.obstacle_set = static_cast<double>(obstacle_set);

        pItem = item(i, 8); //pref_speed
        auto pref_speed = pItem->text().toDouble(&OK_status);
        if(!OK_status) {
            std::cout << "Error in saving pref_speed for Agent Profile: [" 
                << profile_name.toStdString() << "]" << std::endl;
            return -1;            
        }
        current_profile.pref_speed = static_cast<double>(pref_speed);

        pItem = item(i, 9); //r
        auto r = pItem->text().toDouble(&OK_status);
        if(!OK_status) {
            std::cout << "Error in saving r for Agent Profile: [" 
                << profile_name.toStdString() << "]" << std::endl;
            return -1;            
        }        
        current_profile.r = static_cast<double>(r);

        pItem = item(i, 10); //ORCA_tau
        auto ORCA_tau = pItem->text().toDouble(&OK_status);
        if(!OK_status) {
            std::cout << "Error in saving ORCA_tau for Agent Profile: [" 
                << profile_name.toStdString() << "]" << std::endl;
            return -1;            
        }
        current_profile.ORCA_tau = static_cast<double>(ORCA_tau);

        pItem = item(i, 11); //ORCA_tauObst
        auto ORCA_tauObst = pItem->text().toDouble(&OK_status);
        if(!OK_status) {
            std::cout << "Error in saving ORCA_tauObst for Agent Profile: [" 
                << profile_name.toStdString() << "]" << std::endl;
            return -1;            
        }
        current_profile.ORCA_tauObst = static_cast<double>(ORCA_tauObst);   

    }
    return row_count-1;

}

AgentProfileDialog::AgentProfileDialog(CrowdSimImplPtr crowd_sim_impl)
    : CrowdSimDialog(crowd_sim_impl)
{
    agent_profile_tab = std::make_shared<AgentProfileTab>(crowd_sim_impl);
    agent_profile_tab->update();

    setWindowTitle("Agent Profiles");

    QHBoxLayout* table_box = new QHBoxLayout;
    table_box->addWidget(agent_profile_tab.get());

    top_vbox->addLayout(table_box);
    top_vbox->addLayout(bottom_buttons_hbox);
}

void AgentProfileDialog::ok_button_clicked() {
    agent_profile_tab->save();
    accept();
}

//=============================================================
TransitionDialog::TransitionDialog(CrowdSimImplPtr crowd_sim_impl) 
    : CrowdSimDialog(crowd_sim_impl)
{
    transition_tab = std::make_shared<TransitionTab>(crowd_sim_impl);
    transition_tab->update();

    setWindowTitle("Transitions");

    QHBoxLayout* table_box = new QHBoxLayout;
    table_box->addWidget(transition_tab.get());

    top_vbox->addLayout(table_box);
    top_vbox->addLayout(bottom_buttons_hbox);

}


TransitionTab::TransitionTab(CrowdSimImplPtr crowd_sim_impl) 
    : TableList(6), implPtr(crowd_sim_impl)
{
    const QStringList labels =
        { "From State", "To State", "To-State edit" , "Condition", "Condition edit", ""};
    
    label_size = labels.size();
    setHorizontalHeaderLabels(labels);
    setMinimumSize(800, 400);
}

void TransitionTab::update() {
    blockSignals(true);
    clearContents();
    auto transition_number = implPtr->transitions.size();
    setRowCount(1 + transition_number);

    list_transition_in_impl();

    QPushButton* add_button = new QPushButton("Add...", this);
    for (auto i = 0; i < label_size-1; i++) {
        setItem(transition_number, i, new QTableWidgetItem(QString::fromStdString("")));
    }
    setCellWidget(transition_number, label_size-1, add_button);
    connect(
        add_button,
        &QAbstractButton::clicked,
        [&]() {
            add_button_clicked();
            update();
        }
    );

    blockSignals(false);
}

void TransitionTab::list_transition_in_impl() {

    auto transition_number = implPtr->transitions.size();

    for (auto i = 0; i < transition_number; i++) {
        auto& transition = implPtr->transitions[i];

        QComboBox* from_state_comboBox = new QComboBox;
        list_from_states_in_combo(from_state_comboBox, transition);
        setCellWidget(i, 0, from_state_comboBox);

        auto to_state = transition.getToState();
        std::string to_state_name = "";
        for ( auto state : to_state) {
            to_state_name += state.first + ";" ;
        }
        setItem(i, 1, new QTableWidgetItem(QString::fromStdString(to_state_name)));

        QPushButton* to_state_edit = new QPushButton("Edit", this);
        setCellWidget(i, 2, to_state_edit);
        connect(
            to_state_edit,
            &QAbstractButton::clicked,
            [this, &transition](){
                ToStateDialog to_state_dialog(transition, this->implPtr);
                to_state_dialog.exec();
                update();
            }
        );

        auto condition_name = transition.getCondition()->getConditionName();
        setItem(i, 3, new QTableWidgetItem(QString::fromStdString(condition_name)));

        QPushButton* condition_edit = new QPushButton("Edit", this);
        setCellWidget(i, 4, condition_edit);
        connect(
            condition_edit,
            &QAbstractButton::clicked,
            [this, &transition](){
                ConditionDialog condition_dialog(transition, this->implPtr);
                condition_dialog.exec();
                update();
            }
        );

        QPushButton* delete_button = new QPushButton("delete", this);
        setCellWidget(i, 5, delete_button);
        connect(
            delete_button,
            &QAbstractButton::clicked,
            [this, i]() {
                this->implPtr->transitions.erase(implPtr->transitions.begin() + i);
                update();
            }
        );   
    }
}

void TransitionTab::list_from_states_in_combo(QComboBox*& comboBox, crowd_sim::Transition& transition) {
    for (auto state : implPtr->states) {
        if(state.getFinalState()) { //transitions end up with final state
            continue;
        }
        comboBox->addItem(QString::fromStdString(state.getName() ) );
    }
    auto index = comboBox->findText(QString::fromStdString(transition.getFromState()) );
    if (index >= 0) {
        comboBox->setCurrentIndex(index);
    } else {
        comboBox->setCurrentIndex(0);
    }

    connect(
        comboBox,
        QOverload<int>::of(&QComboBox::activated),
        [this, comboBox, &transition](int index) {
            auto state_name = comboBox->itemText(index).toStdString();
            transition.setFromState(state_name);
        }
    );
}

void TransitionTab::save() {
    // instead of clearing the vector first, check the invalid transition and delete it
    update();
    auto row_count = rowCount();
    std::vector<size_t> invalid_trasition;
    for(auto i = 0; i < row_count-1; i++) {
        auto& current_transition = implPtr->transitions.at(i);
        if (!current_transition.isValid() ) {
            invalid_trasition.push_back(i);
        }
    }
    //delete all the invalid transition, from the transitions end to begin
    while(!invalid_trasition.empty()) {
        size_t index = invalid_trasition.back();
        implPtr->transitions.erase(implPtr->transitions.begin() + index);
        invalid_trasition.pop_back();
    }
    update();
}

void TransitionTab::add_button_clicked() {
    save();
    implPtr->transitions.emplace_back("");
}

ToStateDialog::ToStateDialog(crowd_sim::Transition& transition, CrowdSimImplPtr crowd_sim_impl)
    : CrowdSimDialog(crowd_sim_impl)
{
    to_state_tab = std::make_shared<ToStateTab>(transition, crowd_sim_impl);
    to_state_tab->update();

    setWindowTitle("Transition To State Setup" );

    QHBoxLayout* table_box = new QHBoxLayout;
    table_box->addWidget(to_state_tab.get());

    top_vbox->addLayout(table_box);
    top_vbox->addLayout(bottom_buttons_hbox);
}

void ToStateDialog::ok_button_clicked() {
    to_state_tab->save();
    accept();
}

ToStateTab::ToStateTab(crowd_sim::Transition& transition, CrowdSimImplPtr crowd_sim_impl)
    : TableList(3), current_transition(transition), implPtr(crowd_sim_impl)
{   
    const QStringList labels {"To State Name", "Weight", ""};
    label_size = labels.size();
    setHorizontalHeaderLabels(labels);
    setMinimumSize(400, 400);
}

void ToStateTab::update() {
    blockSignals(true);
    clearContents();
    
    auto to_state_number = current_transition.getToState().size();
    setRowCount(1 + to_state_number);
    if (to_state_number != implPtr->states.size()) { 
        // to_states include all the states defined, not allowed to add more
        QPushButton* add_button = new QPushButton("Add...", this);
        for (auto i = 0; i < label_size-1; i++) {
            setItem(to_state_number, i, new QTableWidgetItem(QString::fromStdString("")));
        }
        setCellWidget(to_state_number, label_size-1, add_button);
        connect(
            add_button,
            &QAbstractButton::clicked,
            [&]() {
                add_button_clicked();
                update();
            }
        );
    }

    list_to_states_in_current_transition();

    blockSignals(false);
}

void ToStateTab::list_to_states_in_current_transition() {
    auto to_state_number = current_transition.getToState().size();
    size_t row_number = 0;
    for (auto to_state : current_transition.getToState() ) {
        auto to_state_name = to_state.first;
        auto to_state_weight = to_state.second;

        QComboBox* state_comboBox = new QComboBox;
        for (auto state : implPtr->states) {
            state_comboBox->addItem( QString::fromStdString(state.getName() ));
        }
        auto index = state_comboBox->findText(QString::fromStdString(to_state_name) );
        if (index >= 0) {
            state_comboBox->setCurrentIndex(index);
        } else {
            state_comboBox->setCurrentIndex(0);
        }
        setCellWidget(row_number, 0, state_comboBox);

        setItem(row_number, 1, new QTableWidgetItem(QString::number(to_state_weight)));

        QPushButton* delete_button = new QPushButton("Delete", this);
        setCellWidget(row_number, 2, delete_button);
        connect(
            delete_button,
            &QAbstractButton::clicked,
            [this, to_state_name](){
                this->current_transition.deleteToState(to_state_name);
                update();
            }
        );

        row_number++;
    }
}

void ToStateTab::add_button_clicked() {
    save();
    // insert an empty to_state key
    current_transition.addToState("");
}

void ToStateTab::save() {
    blockSignals(true);
    current_transition.clearToState();
    auto row_count = rowCount() - 1;
    for (auto row = 0; row < row_count; row++) {
        QComboBox* current_to_state_combo = static_cast<QComboBox*>(cellWidget(row, 0));
        std::string to_state_name = current_to_state_combo->currentText().toStdString();

        QTableWidgetItem* weight_item = item(row, 1);
        bool OK_status;
        double to_state_weight = weight_item->text().toDouble(&OK_status);
        if(!OK_status) {
            std::cout << "Error in saving ToStateTab, invalid weight provided!" << std::endl;
            continue;
        }
        current_transition.addToState(to_state_name, to_state_weight);
    }
    blockSignals(false);
}

ConditionDialog::ConditionDialog(crowd_sim::Transition& transition, CrowdSimImplPtr crowd_sim_impl)
    : CrowdSimDialog(crowd_sim_impl), current_transition(transition)
{
    
    setWindowTitle("Transition Condition Setup" );
    // root condition comboBox
    QHBoxLayout* root_condition_type = new QHBoxLayout;
    root_type = new QComboBox;
    root_type->addItem("base_condition(invalid)");
    root_type->addItem("goal_reached");
    root_type->addItem("timer");
    root_type->addItem("and");
    root_type->addItem("or");
    root_type->addItem("not");

    connect(
        root_type,
        QOverload<int>::of(&QComboBox::activated),
        [this](int index) {
            if (index == 0) { //"base_condition(invalid)"
                this->root_condition_value_container->setEnabled(false);
                this->condition1_container->setEnabled(false);
                this->condition2_container->setEnabled(false);
                return;          
            }
            if (index == 1 || index == 2) {//"goal_reached" || "timer"
                this->root_condition_value_container->setEnabled(true);
                this->condition1_container->setEnabled(false);
                this->condition2_container->setEnabled(false);

                if (index == 1 ) { //"goal_reached"
                    if(current_transition.getCondition()->getType() == crowd_sim::Condition::GOAL) return;
                    current_transition.setCondition(
                        static_cast<crowd_sim::ConditionPtr>(std::make_shared<crowd_sim::ConditionGOAL>() ) );
                }
                if (index == 2 ) { //"timer"
                    if(current_transition.getCondition()->getType() == crowd_sim::Condition::TIMER) return;
                    current_transition.setCondition(
                        static_cast<crowd_sim::ConditionPtr>(std::make_shared<crowd_sim::ConditionTIMER>() ) );
                }
                return;
            }
            if (index == 3 || index == 4) { //"and" || "or"
                this->root_condition_value_container->setEnabled(false);
                this->condition1_container->setEnabled(true);
                this->condition2_container->setEnabled(true);

                if (index == 3 ) { //"and"
                    if(current_transition.getCondition()->getType() == crowd_sim::Condition::AND) return;
                    current_transition.setCondition(
                        static_cast<crowd_sim::ConditionPtr>(std::make_shared<crowd_sim::ConditionAND>() ) );
                }
                if (index == 4 ) { //"or"
                    if(current_transition.getCondition()->getType() == crowd_sim::Condition::OR) return;
                    current_transition.setCondition(
                        static_cast<crowd_sim::ConditionPtr>(std::make_shared<crowd_sim::ConditionOR>() ) );
                }
                return;
            }
            if (index == 5 ) { //"not"
                this->root_condition_value_container->setEnabled(false);
                this->condition1_container->setEnabled(true);
                this->condition2_container->setEnabled(false);

                if(current_transition.getCondition()->getType() == crowd_sim::Condition::NOT) return;
                current_transition.setCondition(
                    static_cast<crowd_sim::ConditionPtr>(std::make_shared<crowd_sim::ConditionNOT>() ) );
                return;
            }
        }
    );

    root_condition_type->addWidget(new QLabel("Condition type:"));
    root_condition_type->addWidget(root_type);

    // root condition value container
    root_condition_value_container = new QWidget;
    QHBoxLayout* root_condition_value = new QHBoxLayout(root_condition_value_container);
    root_value = new QLineEdit(QString::number(rootValueD));
    root_condition_value->addWidget(new QLabel("Value (duration(s) / goal distance(m))"));
    root_condition_value->addWidget(root_value);
    connect(
        root_value,
        &QLineEdit::editingFinished,
        [&]() {
            bool OK_status;
            double temp = root_value->text().toDouble(&OK_status);
            if(!OK_status) {
                std::cout << "Invalid condition value input!" << std::endl;
                return;
            }
            auto root_condition = current_transition.getCondition();
            // save to condition
            if (crowd_sim::Condition::GOAL == root_condition->getType()) {
                auto goal_condition = std::dynamic_pointer_cast<crowd_sim::ConditionGOAL>(root_condition);
                goal_condition->setValue(temp);
            }
            if(crowd_sim::Condition::TIMER == root_condition->getType()) {
                auto timer_condition = std::dynamic_pointer_cast<crowd_sim::ConditionTIMER>(root_condition);
                timer_condition->setValue(temp);
            }
        }
    );

    // 2 condition container
    condition1_container = new QWidget;
    construct_leaf_condition_widget(condition1_container, condition1_type, condition1_value, 1);
    condition2_container = new QWidget;
    construct_leaf_condition_widget(condition2_container, condition2_type, condition2_value, 2);

    top_vbox->addLayout(root_condition_type);
    top_vbox->addWidget(root_condition_value_container);
    top_vbox->addWidget(condition1_container);
    top_vbox->addWidget(condition2_container);
    top_vbox->addLayout(bottom_buttons_hbox);

    root_condition_value_container->setEnabled(false);
    condition1_container->setEnabled(false);
    condition2_container->setEnabled(false);

    update();

}

void ConditionDialog::construct_leaf_condition_widget(
    QWidget*& condition_container, QComboBox*& condition_type, QLineEdit*& condition_value, int condition_index) {

    QHBoxLayout* condition_hbox = new QHBoxLayout(condition_container);
    condition_type = new QComboBox;
    condition_type->addItem("goal_reached");
    condition_type->addItem("timer");

    condition_value = new QLineEdit(QString::number(0));
    std::string condition_label = "condition" + std::to_string(condition_index) + ":";
    condition_hbox->addWidget(new QLabel(QString::fromStdString(condition_label) ));
    condition_hbox->addWidget(condition_type);
    condition_hbox->addWidget(condition_value);
    connect(
        condition_value,
        &QLineEdit::editingFinished,
        [this, condition_type, condition_value, condition_index](){
            bool OK_status;
            double temp_value = condition_value->text().toDouble(&OK_status);
            if (!OK_status) {
                std::cout << "Invalid condition value input!" << std::endl;
                return;
            }

            if (condition_type->currentIndex() == 0) { //goal_reached condition
                this->set_sub_condition_in_root_condition(crowd_sim::Condition::GOAL, temp_value, condition_index);
            }
            if (condition_type->currentIndex() == 1) { //timer condition
                this->set_sub_condition_in_root_condition(crowd_sim::Condition::TIMER, temp_value, condition_index);
            }
            
        }
    );
}

void ConditionDialog::set_sub_condition_in_root_condition(
    crowd_sim::Condition::TYPE type, double value, int condition_index) {
    
    crowd_sim::ConditionPtr sub_condition = nullptr;
    if(type == crowd_sim::Condition::GOAL) {
        sub_condition = std::make_shared<crowd_sim::ConditionGOAL>();
        auto temp = std::dynamic_pointer_cast<crowd_sim::ConditionGOAL>(sub_condition);
        temp->setValue(value);
    }
    if(type == crowd_sim::Condition::TIMER) {
        sub_condition = std::make_shared<crowd_sim::ConditionTIMER>();
        auto temp = std::dynamic_pointer_cast<crowd_sim::ConditionTIMER>(sub_condition);
        temp->setValue(value);
    }

    if(!sub_condition) {
        std::cout << "Invalid sub condition!" << std::endl;
        return;
    }

    crowd_sim::ConditionPtr root_condition = this->current_transition.getCondition();
    if (crowd_sim::Condition::AND == root_condition->getType()) {
        crowd_sim::ConditionAndPtr temp = std::dynamic_pointer_cast<crowd_sim::ConditionAND>(root_condition);
        temp->setCondition(sub_condition, condition_index);
    }
    if (crowd_sim::Condition::OR == root_condition->getType()) {
        crowd_sim::ConditionOrPtr temp = std::dynamic_pointer_cast<crowd_sim::ConditionOR>(root_condition);
        temp->setCondition(sub_condition, condition_index);
    }
    if (crowd_sim::Condition::NOT == root_condition->getType()) {
        crowd_sim::ConditionNotPtr temp = std::dynamic_pointer_cast<crowd_sim::ConditionNOT>(root_condition);
        temp->setCondition(sub_condition);
    }
    
}

void ConditionDialog::initialize_sub_condition(
    crowd_sim::ConditionPtr sub_condition, double value) {
    if (!sub_condition) {
        std::cout << "Invalid sub condition!" << std::endl;
        return;
    }
    
    if (crowd_sim::Condition::GOAL == sub_condition->getType()) { //goal_reached condition
        crowd_sim::ConditionGoalPtr temp = std::dynamic_pointer_cast<crowd_sim::ConditionGOAL>(sub_condition);
        if (value > 0) temp->setValue(value);
    }
    if (crowd_sim::Condition::TIMER == sub_condition->getType()) { //timer condition
        crowd_sim::ConditionTimerPtr temp = std::dynamic_pointer_cast<crowd_sim::ConditionTIMER>(sub_condition);
        if (value > 0) temp->setValue(value);
    }
    if (!sub_condition) {
        std::cout << "Empty sub condition!" << std::endl;
        return;
    }
}

void ConditionDialog::ok_button_clicked() {
    save();
    accept();
}

void ConditionDialog::save() {

    if (root_condition_value_container->isEnabled()) {
        root_value->editingFinished();
    }
    if (condition1_container->isEnabled()) {
        condition1_value->editingFinished();
    }
    if (condition2_container->isEnabled()) {
        condition2_value->editingFinished();
    }
}

void ConditionDialog::update() {
    crowd_sim::ConditionPtr root_condition = current_transition.getCondition();

    switch (root_condition->getType()) {
        case crowd_sim::Condition::GOAL : 
            root_type->setCurrentIndex(1);
            break;
        case crowd_sim::Condition::TIMER :
            root_type->setCurrentIndex(2);
            break;
        case crowd_sim::Condition::AND :
            root_type->setCurrentIndex(3);
            break;
        case crowd_sim::Condition::OR :
            root_type->setCurrentIndex(4);
            break;
        case crowd_sim::Condition::NOT :
            root_type->setCurrentIndex(5);
            break;
        default:
            root_type->setCurrentIndex(0);
    }

    if (crowd_sim::Condition::GOAL == root_condition->getType()) {
        auto goal_condition = std::dynamic_pointer_cast<crowd_sim::ConditionGOAL>(root_condition);
        rootValueD = goal_condition->getValue();
    }
    if(crowd_sim::Condition::TIMER == root_condition->getType()) {
        auto timer_condition = std::dynamic_pointer_cast<crowd_sim::ConditionTIMER>(root_condition);
        rootValueD = timer_condition->getValue();
    }
    root_value->setText(QString::number(rootValueD));

    if(crowd_sim::Condition::NOT == root_condition->getType()) {
        auto not_condition = std::dynamic_pointer_cast<crowd_sim::ConditionNOT>(root_condition);
        auto sub_condition = not_condition->getCondition();
        if (crowd_sim::Condition::GOAL == sub_condition->getType()) {
            condition1ValueD = std::dynamic_pointer_cast<crowd_sim::ConditionGOAL>(sub_condition)->getValue();
        }
        if (crowd_sim::Condition::TIMER == sub_condition->getType()) {
            condition1ValueD = std::dynamic_pointer_cast<crowd_sim::ConditionTIMER>(sub_condition)->getValue();
        }
    }

    if(crowd_sim::Condition::AND == root_condition->getType() ) {
        auto and_condition = std::dynamic_pointer_cast<crowd_sim::ConditionAND>(root_condition);
        for (int i = 1; i <= 2; i++) {
            auto sub_condition = and_condition->getCondition(i);
            double temp_value;
            if (crowd_sim::Condition::GOAL == sub_condition->getType()) {
                temp_value = std::dynamic_pointer_cast<crowd_sim::ConditionGOAL>(sub_condition)->getValue();
            }
            if (crowd_sim::Condition::TIMER == sub_condition->getType()) {
                temp_value = std::dynamic_pointer_cast<crowd_sim::ConditionTIMER>(sub_condition)->getValue();
            }
            if (i == 1) {
                condition1ValueD = temp_value;
            }
            if (i == 2) {
                condition2ValueD = temp_value;
            }
        }
    }

    if(crowd_sim::Condition::OR == root_condition->getType() ) {
        auto or_condition = std::dynamic_pointer_cast<crowd_sim::ConditionOR>(root_condition);
        for (int i = 1; i <= 2; i++) {
            auto sub_condition = or_condition->getCondition(i);
            double temp_value;
            if (crowd_sim::Condition::GOAL == sub_condition->getType()) {
                temp_value = std::dynamic_pointer_cast<crowd_sim::ConditionGOAL>(sub_condition)->getValue();
            }
            if (crowd_sim::Condition::TIMER == sub_condition->getType()) {
                temp_value = std::dynamic_pointer_cast<crowd_sim::ConditionTIMER>(sub_condition)->getValue();
            }
            if (i == 1) {
                condition1ValueD = temp_value;
            }
            if (i == 2) {
                condition2ValueD = temp_value;
            }
        }
    }

    condition1_value->setText(QString::number(condition1ValueD));
    condition2_value->setText(QString::number(condition2ValueD));
}

//============================================================================
AgentGroupDialog::AgentGroupDialog(CrowdSimImplPtr crowd_sim_impl)
    : CrowdSimDialog(crowd_sim_impl)
{
    agent_group_tab = std::make_shared<AgentGroupTab>(crowd_sim_impl);
    agent_group_tab->update();

    setWindowTitle("Agent Group");

    QHBoxLayout* table_box = new QHBoxLayout;
    table_box->addWidget(agent_group_tab.get());

    top_vbox->addLayout(table_box);
    top_vbox->addLayout(bottom_buttons_hbox);
}

AgentGroupTab::AgentGroupTab(CrowdSimImplPtr crowd_sim_impl) 
    : TableList(8), implPtr(crowd_sim_impl)
{
    const QStringList labels = {
        "id", "profile", "initial state", "spawn number", "external agent", "point_x", "point_y", ""
    };
    label_size = labels.size();
    setHorizontalHeaderLabels(labels);
    horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    setMinimumSize(1200, 400);
}

void AgentGroupTab::update() {
    blockSignals(true);
    clearContents();
    auto group_number = implPtr->agent_groups.size();
    setRowCount(1 + group_number);

    list_agent_group_in_impl();

    QPushButton* add_button = new QPushButton("Add...", this);
    for (auto i = 0; i < label_size-1; i++) {
        setItem(group_number, i, new QTableWidgetItem(QString::fromStdString("")));
    }
    setCellWidget(group_number, label_size-1, add_button);
    connect(
        add_button,
        &QAbstractButton::clicked,
        [&]() {
            add_button_clicked();
            update();
        }
    );

    blockSignals(false);
}

void AgentGroupTab::save() {
    auto row_count = rowCount();
    //row 0 reserved for external agent, only save the agent profile and initial state
    auto& external_group = implPtr->agent_groups.at(0);
    auto profile_combo = static_cast<QComboBox*>(cellWidget(0, 1));
    external_group.setAgentProfile(profile_combo->currentText().toStdString());
    auto state_combo = static_cast<QComboBox*>(cellWidget(0, 2));
    external_group.setInitialState(state_combo->currentText().toStdString());

    for (auto row = 1; row < row_count-1; row++) {
        auto& current_group = implPtr->agent_groups.at(row);
        auto profile_combo = static_cast<QComboBox*>(cellWidget(row, 1));
        current_group.setAgentProfile(profile_combo->currentText().toStdString());
        auto state_combo = static_cast<QComboBox*>(cellWidget(row, 2));
        current_group.setInitialState(state_combo->currentText().toStdString());
        
        bool OK_status;
        int spawn_number = item(row, 3)->text().toInt(&OK_status);
        if (!OK_status) {
            std::cout << "Invalid input of spawn_number, use default 0." << std::endl;
            spawn_number = 0;
        }
        current_group.setSpawnNumber(spawn_number);

        double point_x = item(row, 5)->text().toDouble(&OK_status);
        if (!OK_status) {
            std::cout << "Invalid input of x for spawn point coord, use default 0.0." << std::endl;
            point_x = 0.0;
        }
        double point_y = item(row, 6)->text().toDouble(&OK_status);
        if (!OK_status) {
            std::cout << "Invalid input of y for spawn point coord, use default 0.0." << std::endl;
            point_y = 0.0;
        }
        current_group.setSpawnPoint(point_x, point_y);
    }

}

void AgentGroupTab::add_button_clicked() {
    save();
    size_t new_id = 0;
    if(implPtr->agent_groups.size() > 0) {
        new_id = implPtr->agent_groups.back().getGroupId() + 1;
    }
    implPtr->agent_groups.emplace_back(new_id); //no external agent group added manually
}

void AgentGroupTab::list_agent_group_in_impl() {
    auto group_number = implPtr->agent_groups.size();

    for (auto row = 0; row < group_number; row++) {
        auto& current_group = implPtr->agent_groups.at(row);
        
        setItem(row, 0, new QTableWidgetItem(QString::number(static_cast<int>(current_group.getGroupId()) )));
        auto current_profile = current_group.getAgentProfile();
        QComboBox* profile_combo = new QComboBox;
        add_profiles_in_combobox(profile_combo, current_profile);
        setCellWidget(row, 1, profile_combo);

        auto current_state = current_group.getInitialState();
        QComboBox* state_combo = new QComboBox;
        add_states_in_combobox(state_combo, current_state);
        setCellWidget(row, 2, state_combo);
        setItem(row, 3, new QTableWidgetItem(QString::number(current_group.getSpawnNumber() )));

        std::string external_agent_name = "";
        if (current_group.isExternalGroup()) {
            for (auto name : current_group.getExternalAgentName()) {
                external_agent_name += name + ";";
            }
        }

        setItem(row, 4, new QTableWidgetItem(QString::fromStdString(external_agent_name) ));
        auto spawn_point = current_group.getSpawnPoint();
        setItem(row, 5, new QTableWidgetItem(QString::number(spawn_point.first)));
        setItem(row, 6, new QTableWidgetItem(QString::number(spawn_point.second)));

        if(row == 0) {
            // row 0 is not allowed to be deleted
            setItem(row, 7, new QTableWidgetItem(QString::fromStdString("") ));
            continue;
        }

        QPushButton* delete_button = new QPushButton("Delete", this);
        setCellWidget(row, 7, delete_button);
        connect(
            delete_button,
            &QAbstractButton::clicked,
            [this, row]() {
                //delete part
                save();
                if (row == 0) return;
                this->implPtr->agent_groups.erase(this->implPtr->agent_groups.begin() + row);
                update();
            }
        );
    }
}

void AgentGroupTab::add_profiles_in_combobox(QComboBox*& profile_combo, std::string current_profile) {
    for (auto profile : implPtr->agent_profiles) {
        profile_combo->addItem(QString::fromStdString(profile.profile_name));
    }
    int current_index = profile_combo->findText(QString::fromStdString(current_profile));
    if (current_index > 0) {
        profile_combo->setCurrentIndex(current_index);
    } else {
        profile_combo->setCurrentIndex(0);
    }
}

void AgentGroupTab::add_states_in_combobox(QComboBox*& state_combo, std::string current_state) {
    for (auto state : implPtr->states) {
        state_combo->addItem(QString::fromStdString(state.getName()));
    }
    int current_index = state_combo->findText(QString::fromStdString(current_state));
    if (current_index > 0) {
        state_combo->setCurrentIndex(current_index);
    } else {
        state_combo->setCurrentIndex(0);
    }
}
