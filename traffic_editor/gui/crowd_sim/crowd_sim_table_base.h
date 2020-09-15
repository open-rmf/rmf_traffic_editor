#ifndef CROWD_SIM_CROWDSIM_TABLE_BASE__H
#define CROWD_SIM_CROWDSIM_TABLE_BASE__H

#include <memory>

#include <traffic_editor/crowd_sim/crowd_sim_impl.h>

#include "table_list.h"

using namespace crowd_sim;

class CrowdSimTableBase : public TableList
{
public:
  CrowdSimTableBase(CrowdSimImplPtr impl, const QStringList& labels);
  virtual ~CrowdSimTableBase() {}

  CrowdSimImplPtr get_impl() const { return _crowd_sim_impl; }
  size_t get_label_size() const { return _label_size; }
  void set_label_size(size_t label_size) { _label_size = label_size; }

  virtual int get_cache_size() const = 0;
  virtual void list_item_in_cache() = 0;
  virtual void save() = 0;
  virtual void save_to_impl() = 0;
  virtual void add_button_click() = 0;
  virtual void delete_button_click(size_t row_num) = 0;

  virtual void update();

private:
  CrowdSimImplPtr _crowd_sim_impl;
  size_t _label_size;
};

using CrowdSimTablePtr = std::shared_ptr<CrowdSimTableBase>;

#endif