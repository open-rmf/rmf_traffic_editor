#ifndef CROWD_SIM_EDITOR_TABLE__H
#define CROWD_SIM_EDITOR_TABLE__H

#include <vector>
#include <string>
#include <set>

#include <QTableWidget>
#include <QtWidgets>

#include "table_list.h"
#include "project.h"
#include <traffic_editor/crowd_sim/crowd_sim_impl.h>

using namespace crowd_sim;

class CrowdSimEditorTable : public TableList
{
  Q_OBJECT;

public:
  CrowdSimEditorTable(const Project& input_project);
  ~CrowdSimEditorTable() {}

  void update();
  void update_goal_area();
  void update_navmesh_level();
  void update_external_agent_from_spawn_point();
  void update_external_agent_state();

private:
  const Project& _project;
  CrowdSimImplPtr _impl;

  int _reserved_rows = 2; // checkbox for enable_crowd_sim, LineEdit for updtae_time_step
  std::vector<std::string> _required_components {
    "GoalSets",
    "States",
    "Transitions",
    "AgentProfiles",
    "AgentGroups",
    "ModelTypes"};
  std::set<std::string> _goal_areas_cache;
  std::vector<std::string> _navmesh_filename_cache;

  QTableWidgetItem* _enable_crowd_sim_name_item;
  QCheckBox* _enable_crowd_sim_checkbox;
  QTableWidgetItem* _update_time_step_name_item;
  QLineEdit* _update_time_step_value_item;
};

#endif
