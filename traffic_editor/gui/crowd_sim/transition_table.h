#ifndef CROWD_SIM_TRANSITION_TABLE__H
#define CROWD_SIM_TRANSITION_TABLE__H

#include <traffic_editor/crowd_sim/crowd_sim_impl.h>

#include "crowd_sim_table_base.h"

using namespace crowd_sim;

class TransitionTab : public CrowdSimTableBase
{
public:
  static std::shared_ptr<TransitionTab> init_and_make(CrowdSimImplPtr crowd_sim_impl);

  TransitionTab(CrowdSimImplPtr crowd_sim_impl, const QStringList& labels)
    : CrowdSimTableBase(crowd_sim_impl, labels)
  {
    _cache = get_impl()->get_transitions();
  }
  ~TransitionTab() {}

  int get_cache_size() const override { return static_cast<int>(_cache.size()); }
  void list_item_in_cache() override;
  void save() override;
  void save_to_impl() override;
  void add_button_click() override;
  void delete_button_click(size_t row_number) override;

private:
  std::vector<crowd_sim::Transition> _cache;

  void _list_from_states_in_combo(
    QComboBox* comboBox,
    crowd_sim::Transition& transition);
};

#endif