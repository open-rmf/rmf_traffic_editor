#ifndef CROWD_SIM_TO_STATE_TABLE__H
#define CROWD_SIM_TO_STATE_TABLE__H

#include <traffic_editor/crowd_sim/crowd_sim_impl.h>
#include "crowd_sim_table_base.h"

using namespace crowd_sim;

class ToStateTab : public CrowdSimTableBase
{
public:
  static std::shared_ptr<ToStateTab> init_and_make(
    CrowdSimImplPtr crowd_sim_impl, 
    crowd_sim::Transition& transition);

  ToStateTab(
    CrowdSimImplPtr crowd_sim_impl, 
    crowd_sim::Transition& transition, 
    const QStringList& labels)
    : CrowdSimTableBase(crowd_sim_impl, labels),
      _current_transition(transition)
  {
    _cache.clear();
    for (auto to_state : transition.get_to_state())
    _cache.emplace_back(to_state);
  }
  ~ToStateTab() {}

  int get_cache_size() const override { return static_cast<int>(_cache.size()); }
  void list_item_in_cache() override;
  void save() override;
  void save_to_impl() override;
  void add_button_click() override;
  void delete_button_click(size_t row_number) override;

private:
  //to_state_cache<to_state_name, weight>
  std::vector<std::pair<std::string, double>> _cache;
  Transition& _current_transition;

};

#endif