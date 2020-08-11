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

    for (size_t i = 0; i < states_number; i++) {
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
    implPtr->states.emplace_back("state" + std::to_string( save() ));
}

int StatesTab::save() {
    auto rows_count = rowCount();
    for (auto i = 0; i < rows_count - 1; i++) {
        auto& current_state = implPtr->states.at(i);
        auto name_item = item(i, 0)->text().toStdString();
        current_state.setName(name_item);
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
    auto row_count = save();
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
    implPtr->clearAgentProfile();
    //ignore the first row of external agent, which has been added in clear agent profile
    for (auto i = 1; i < row_count - 1; i++) {
        QTableWidgetItem* pItem = item(i, 0);
        auto profile_name = pItem->text();
        implPtr->agent_profiles.emplace_back(profile_name.toStdString());
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

