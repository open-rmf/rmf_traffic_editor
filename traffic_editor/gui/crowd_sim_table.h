#ifndef CROWD_SIM_TABLE__H
#define CROWD_SIM_TABLE__H

#include <vector>
#include <string>
#include <set>

#include <QTableWidget>

#include <table_list.h>
#include <project.h>
#include <crowd_sim_impl.h>

class CrowdSimTable : public TableList
{
    Q_OBJECT;

public:
    CrowdSimTable(const Project& input_project);
    ~CrowdSimTable();

    void update();
    void update_goal_area();

private:
    bool enable_crowd_sim = false;

    std::vector<std::string> required_components { 
        "States", 
        "GoalSets", 
        "Transitions", 
        "AgentProfiles", 
        "AgentGroups", 
        "ModelTypes" };
    std::set<std::string> goal_areas;

    CrowdSimImplPtr crowd_sim_impl;
    const Project& project;

};



#endif
