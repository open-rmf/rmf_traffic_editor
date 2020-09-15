#ifndef CROWD_SIM_DIALOG__H
#define CROWD_SIM_DIALOG__H

#include <memory>

#include <QDialog>
#include <QComboBox>
#include <QtWidgets>

#include <traffic_editor/crowd_sim/crowd_sim_impl.h>

#include "crowd_sim_table_base.h"

using namespace crowd_sim;

class CrowdSimDialog : public QDialog
{
public:
  CrowdSimDialog(
    CrowdSimImplPtr crowd_sim_impl,
    const std::string& dialog_title);
  virtual ~CrowdSimDialog() {}

  QPushButton* ok_button, * cancel_button;
  QHBoxLayout* bottom_buttons_hbox;
  QVBoxLayout* top_vbox;

  virtual void ok_button_click();
  void cancel_button_click();

private:
  CrowdSimTablePtr _table_ptr;
};

#endif