#ifndef CROWD_SIM_GOAL_SET_TABLE__H
#define CROWD_SIM_GOAL_SET_TABLE__H

#include <memory>
#include <vector>

#include <traffic_editor/crowd_sim/crowd_sim_impl.h>
#include <traffic_editor/crowd_sim/goal_set.h>

#include "crowd_sim_table_base.h"

using namespace crowd_sim;

class GoalSetTab : public CrowdSimTableBase
{
public:
  static std::shared_ptr<GoalSetTab> init_and_make(CrowdSimImplPtr crowd_sim_impl);
  
  GoalSetTab(CrowdSimImplPtr crowd_sim_impl, const QStringList& labels)
    : CrowdSimTableBase(crowd_sim_impl, labels)
  {
    _cache = get_impl()->get_goal_sets();
  }
  ~GoalSetTab() {}

  int get_cache_size() const override { return static_cast<int>(_cache.size()); }
  void list_item_in_cache() override;
  void save() override;
  void save_to_impl() override;
  void add_button_click() override;
  void delete_button_click(size_t row_number) override;

private:
  std::vector<crowd_sim::GoalSet> _cache;
};

#endif