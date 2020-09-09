#include <QHeaderView>
#include <QWidget>
#include <QPushButton>

#include "crowd_sim_table_base.h"

//========================================
CrowdSimTableBase::CrowdSimTableBase(CrowdSimImplPtr impl,
  const QStringList& labels)
: TableList(labels.size()),
  _crowd_sim_impl(impl),
  _label_size(labels.size())
{
  setHorizontalHeaderLabels(labels);
  horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
  setMinimumSize(800, 400);
  setSizePolicy(
    QSizePolicy::Expanding,
    QSizePolicy::MinimumExpanding);
}

//======================================
void CrowdSimTableBase::update()
{
  blockSignals(true);
  clearContents();

  int cache_item_size = get_cache_size();

  setRowCount(
    cache_item_size +
    1   // put add button in this row
  );
  list_item_in_cache();

  for (auto i = 0; i < cache_item_size; i++)
  {
    QPushButton* delete_button = new QPushButton("Del");
    setCellWidget(i, get_label_size() - 1, delete_button);
    connect(
      delete_button,
      &QAbstractButton::clicked,
      [i, this]()
      {
        save();
        delete_button_click(i);
        update();
      }
    );
  }

  QPushButton* add_button = new QPushButton("Add");
  setCellWidget(cache_item_size, get_label_size() - 1, add_button);
  connect(
    add_button,
    &QAbstractButton::clicked,
    [this]()
    {
      save();
      add_button_click();
      update();
    }
  );

  blockSignals(false);
}
