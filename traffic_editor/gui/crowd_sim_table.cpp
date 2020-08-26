#include "crowd_sim_table.h"
#include "crowd_sim_dialog.h"

#include <iostream>

#include <QString>


CrowdSimTable::CrowdSimTable(const Project& input_project) : TableList(3), project(input_project)
{
    if (project.building.crowd_sim_impl == nullptr) {
        printf("Initialize crowd_sim_implementation for project.building\n");
        crowd_sim_impl = std::make_shared<crowd_sim::CrowdSimImplementation>();
        project.building.crowd_sim_impl = crowd_sim_impl;
    } else {
        crowd_sim_impl = project.building.crowd_sim_impl;
    }

    const QStringList labels =
    { "Name", "Status", "" };
    setHorizontalHeaderLabels(labels);

    // 2: enable_crowd_sim and update_time_step 
    setRowCount(2 + this->required_components.size());

    // enable_crowd_sim check box
    enable_crowd_sim_name_item = new QTableWidgetItem(QString::fromStdString("enable_crowd_sim"));
    setItem(0, 0, enable_crowd_sim_name_item);

    enable_crowd_sim_checkbox = new QCheckBox;
    enable_crowd_sim_checkbox->setChecked(crowd_sim_impl->enable_crowd_sim);
    setCellWidget(0, 1, enable_crowd_sim_checkbox);
    connect(
        enable_crowd_sim_checkbox,
        &QAbstractButton::clicked,
        [this] (bool box_checked){
            crowd_sim_impl->enable_crowd_sim = box_checked;
        }
    );

    // update_time_step
    update_time_step_name_item = new QTableWidgetItem(QString::fromStdString("update_time_step"));
    setItem(1, 0, update_time_step_name_item);

    update_time_step_value_item = new QLineEdit(QString::number(crowd_sim_impl->update_time_step));
    setCellWidget(1, 1, update_time_step_value_item);
    connect(
        update_time_step_value_item,
        &QLineEdit::editingFinished,
        [this]() {
            bool OK_status;
            double update_time_step_ = update_time_step_value_item->text().toDouble(&OK_status);
            if(OK_status) {
                crowd_sim_impl->update_time_step = update_time_step_;
            } else {
                crowd_sim_impl->update_time_step = 0.1;
                std::cout << "Invalid update_time_step provided for crowd_sim. default 0.1s will be used." << std::endl;
            }
        }
    );

    // update required components
    update();
    for (size_t i = 0; i < this->required_components.size(); ++i) {
        int row_id = i+2;
        QTableWidgetItem* name_item = 
            new QTableWidgetItem (QString::fromStdString(this->required_components[i]));
        setItem(row_id, 0, name_item);

        QPushButton* edit_button = new QPushButton("Edit");
        setCellWidget(row_id, 2, edit_button);
        edit_button->setStyleSheet("QTableWidgetItem { background-color: red; }");

        if ("States" == this->required_components[i]) {
            connect(
                edit_button,
                &QAbstractButton::clicked,
                [this](){
                    update();
                    StatesDialog states_dialog(this->crowd_sim_impl);
                    states_dialog.exec();
                    update();
                }
            );
        }
        if ("GoalSets" == this->required_components[i]) {
            connect(
                edit_button,
                &QAbstractButton::clicked,
                [this]() {
                    update();
                    GoalSetDialog goal_set_dialog(this->crowd_sim_impl);
                    goal_set_dialog.exec();
                    update();
                }
            );
        }
        if ("AgentProfiles" == this->required_components[i]) {
            connect(
                edit_button,
                &QAbstractButton::clicked,
                [this]() {
                    update();
                    AgentProfileDialog agent_profile_dialog(this->crowd_sim_impl);
                    agent_profile_dialog.exec();
                    update();
                }
            );
        }
        if ("Transitions" == this->required_components[i]) {
            connect(
                edit_button,
                &QAbstractButton::clicked,
                [this]() {
                    update();
                    TransitionDialog transition_dialog(this->crowd_sim_impl);
                    transition_dialog.exec();
                    update();
                }
            );
        }
        if ("AgentGroups" == this->required_components[i]) {
            connect(
                edit_button,
                &QAbstractButton::clicked,
                [this]() {
                    update();
                    AgentGroupDialog agent_group_dialog(this->crowd_sim_impl);
                    agent_group_dialog.exec();
                    update();
                }
            );
        }
        if ("ModelTypes" == this->required_components[i]) {
            connect(
                edit_button,
                &QAbstractButton::clicked,
                [this]() {
                    update();
                    ModelTypeDialog model_type_dialog(this->crowd_sim_impl);
                    model_type_dialog.exec();
                    update();
                }
            );
        }
    }

}

CrowdSimTable::~CrowdSimTable() {}

void CrowdSimTable::update()
{   
    update_goal_area();
    update_navmesh_level();
    update_external_agent_from_spawn_point();
    update_external_agent_state();

    blockSignals(true);

    enable_crowd_sim_checkbox->setChecked(crowd_sim_impl->enable_crowd_sim);
    update_time_step_value_item->setText(QString::number(crowd_sim_impl->update_time_step));

    size_t status_number = 0;
    for(size_t i = 0; i < required_components.size(); ++i){
        status_number = 0;
        if ("States" == this->required_components[i]) {
            status_number = crowd_sim_impl->states.size();
        }
        if ("GoalSets" == this->required_components[i]) {
            status_number = crowd_sim_impl->goal_sets.size();
        }
        if ("AgentProfiles" == this->required_components[i]) {
            status_number = crowd_sim_impl->agent_profiles.size();
        }
        if ("Transitions" == this->required_components[i]) {
            status_number = crowd_sim_impl->transitions.size();
        }
        if ("AgentGroups" == this->required_components[i]) {
            status_number = crowd_sim_impl->agent_groups.size();
        }
        if ("ModelTypes" == this->required_components[i]) {
            status_number = crowd_sim_impl->model_types.size();
        }

        setItem(i+2, 1, new QTableWidgetItem(QString::number(status_number)));
    }

    blockSignals(false);
}

void CrowdSimTable::update_goal_area(){
    crowd_sim_impl->goal_areas.clear();
    for (auto level : project.building.levels) {
        auto vertex_list = level.vertices;
        for (auto vertex : vertex_list){
            if (vertex.params.find("human_goal_set_name") == vertex.params.end() ) continue;
            auto param = vertex.params["human_goal_set_name"];
            if (param.type != param.STRING){
                std::cout << "Error param type for human_goal_set_name." << std::endl;
                return;
            }
            crowd_sim_impl->goal_areas.insert(param.value_string);
        }
    }
}

void CrowdSimTable::update_navmesh_level() {
    crowd_sim_impl->navmesh_filename_list.clear();
    for (auto level : project.building.levels) {
        crowd_sim_impl->navmesh_filename_list.emplace_back(level.name + "_navmesh.nav");
    }
}

void CrowdSimTable::update_external_agent_from_spawn_point() {
    std::vector<std::string> spawn_point_name;
    // currently only for 1 level!
    for (auto level : project.building.levels) {
        for (auto vertex : level.vertices) {
            if (vertex.params.find("spawn_robot_name") != vertex.params.end()) {
                spawn_point_name.emplace_back(vertex.params["spawn_robot_name"].value_string);
            }
        }
    }
    if(crowd_sim_impl->agent_groups.size() == 0) {
        crowd_sim_impl->agent_groups.emplace_back(0, true);
    }
    auto& external_group = crowd_sim_impl->agent_groups.at(0);
    external_group.setExternalAgentName(spawn_point_name);
}

void CrowdSimTable::update_external_agent_state() {
    if(crowd_sim_impl->states.size() == 0) {
        crowd_sim_impl->states.emplace_back("external_static");
    }
    auto& external_state = crowd_sim_impl->states.at(0);
    external_state.setFinalState(true);
}
