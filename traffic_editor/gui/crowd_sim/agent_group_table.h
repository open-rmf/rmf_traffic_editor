#ifndef CROWD_SIM_AGENT_GROUP_TABLE__H
#define CROWD_SIM_AGENT_GROUP_TABLE__H

#include <QComboBox>

#include <traffic_editor/crowd_sim/crowd_sim_impl.h>

#include "crowd_sim_table_base.h"

using namespace crowd_sim;

class AgentGroupTab : public CrowdSimTableBase
{
public:

  static std::shared_ptr<AgentGroupTab> init_and_make(
    CrowdSimImplPtr crowd_sim_impl);

  AgentGroupTab(CrowdSimImplPtr crowd_sim_impl, const QStringList& labels)
  : CrowdSimTableBase(crowd_sim_impl, labels)
  {
    _cache = get_impl()->get_agent_groups();
  }

  ~AgentGroupTab() {}

  int get_cache_size() const override
  {
    return static_cast<int>(_cache.size());
  }
  void list_item_in_cache() override;
  void save() override;
  void save_to_impl() override;
  void add_button_click() override;
  void delete_button_click(size_t row_number) override;

private:
  std::vector<AgentGroup> _cache;
  void _add_profiles_in_combobox(QComboBox* profile_combo,
    std::string current_profile);
  void _add_states_in_combobox(QComboBox* states_combo,
    std::string current_state);
};

#endif