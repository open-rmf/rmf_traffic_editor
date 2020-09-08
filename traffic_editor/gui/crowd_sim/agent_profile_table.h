#ifndef CROWD_SIM_AGENT_PROFILE_TABLE__H
#define CROWD_SIM_AGENT_PROFILE_TABLE__H

#include <table_list.h>
#include <traffic_editor/crowd_sim/crowd_sim_impl.h>

using namespace crowd_sim;

class AgentProfileTab : public TableList
{
public:
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

#endif