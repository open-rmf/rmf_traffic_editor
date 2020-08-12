#include "crowd_sim_table.h"
#include "crowd_sim_dialog.h"

#include <iostream>

#include <QtWidgets>
#include <QString>


CrowdSimTable::CrowdSimTable(const Project& input_project) : TableList(3), project(input_project)
{
    crowd_sim_impl = std::make_shared<crowd_sim::CrowdSimImplementation>();
    update();

    const QStringList labels =
    { "Name", "Status", "" };
    setHorizontalHeaderLabels(labels);

    setRowCount(1 + this->required_components.size());

    // enable_crowd_sim check box
    QTableWidgetItem* enable_crowd_sim_name_item =
      new QTableWidgetItem(QString::fromStdString("enable_crowd_sim"));
    setItem(0, 0, enable_crowd_sim_name_item);

    QCheckBox* enable_crowd_sim_checkbox = new QCheckBox;
    enable_crowd_sim_checkbox->setChecked(this->enable_crowd_sim);
    setCellWidget(0, 1, enable_crowd_sim_checkbox);
    connect(
        enable_crowd_sim_checkbox,
        &QAbstractButton::clicked,
        [this] (bool box_checked){
            this->enable_crowd_sim = box_checked;
        }
    );

    // required components
    for (size_t i = 0; i < this->required_components.size(); ++i) {
        QTableWidgetItem* name_item = 
            new QTableWidgetItem (QString::fromStdString(this->required_components[i]));
        setItem(i+1, 0, name_item);

        QPushButton* edit_button = new QPushButton("Edit");
        setCellWidget(i+1, 2, edit_button);
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
    }

}

CrowdSimTable::~CrowdSimTable() {}

void CrowdSimTable::update()
{   
    update_goal_area();
    update_navmesh_level();

    blockSignals(true);

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

        setItem(i+1, 1, new QTableWidgetItem(QString::number(status_number)));
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
