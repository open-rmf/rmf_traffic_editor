#include <QWidget>
#include <QComboBox>
#include <QPushButton>

#include "transition_table.h"
#include "crowd_sim_dialog.h"
#include "condition_dialog.h"

//==================================================
std::shared_ptr<TransitionTab> TransitionTab::init_and_make(CrowdSimImplPtr crowd_sim_impl) {
  const QStringList labels = 
    { "From State", "To State", "To-State edit", "Condition", "Condition edit", ""};

  auto transition_tab_ptr = std::make_shared<TransitionTab>(crowd_sim_impl, labels);
  if (!transition_tab_ptr) {
    printf("Failed to create goal set table! Exiting");
    return nullptr;
  }
  return transition_tab_ptr;
}

//==================================================
void TransitionTab::list_item_in_cache()
{
  auto cache_count = _cache.size();
  for (size_t i = 0; i < cache_count; i++)
  {
    auto& transition = _cache.at(i);

    QComboBox* from_state_comboBox = new QComboBox;
    _list_from_states_in_combo(from_state_comboBox, transition);
    setCellWidget(i, 0, from_state_comboBox);

    auto to_state = transition.get_to_state();
    std::string to_state_name = "";
    for (auto state : to_state)
    {
      to_state_name += state.first + ";";
    }
    setItem(i, 1, new QTableWidgetItem(QString::fromStdString(to_state_name)));

    QPushButton* to_state_edit = new QPushButton("Edit", this);
    setCellWidget(i, 2, to_state_edit);
    connect(
      to_state_edit,
      &QAbstractButton::clicked,
      [this, &transition]()
      {
        ToStateDialog to_state_dialog(get_impl(), transition);
        to_state_dialog.exec();
        update();
      }
    );

    auto condition_name = transition.get_condition()->get_condition_name();
    setItem(i, 3, new QTableWidgetItem(QString::fromStdString(condition_name)));

    QPushButton* condition_edit = new QPushButton("Edit", this);
    setCellWidget(i, 4, condition_edit);
    connect(
      condition_edit,
      &QAbstractButton::clicked,
      [this, &transition]()
      {
        ConditionDialog condition_dialog(transition);
        condition_dialog.exec();
        update();
      }
    );
  }
}

//==================================================
void TransitionTab::_list_from_states_in_combo(
  QComboBox* comboBox,
  crowd_sim::Transition& transition)
{
  for (auto state : get_impl()->get_states())
  {
    if (state.get_final_state())
    {
      continue;
    }
    comboBox->addItem(QString::fromStdString(state.get_name() ) );
  }
  auto index =
    comboBox->findText(QString::fromStdString(transition.get_from_state()) );
  comboBox->setCurrentIndex(index >= 0 ? index : 0);
}

//==================================================
void TransitionTab::save()
{
  auto tmp_cache = _cache;
  auto row_count = rowCount();
  for (auto i = 0; i < row_count-1; i++)
  {
    auto& current_transition = tmp_cache.at(i);

    auto pItem_from_state = static_cast<QComboBox*>(cellWidget(i, 0));
    current_transition.set_from_state(
      pItem_from_state->currentText().toStdString());
  }
  _cache = tmp_cache;
  update();
}

//==================================================
void TransitionTab::save_to_impl()
{ 
  save();
  std::vector<size_t> invalid_trasition;
  for (size_t i = 0; i < _cache.size(); i++) {
    if (!_cache[i].is_valid()) {
      std::cout << _cache[i].get_from_state() << _cache[i].get_to_state().size() << _cache[i].get_condition()->get_condition_name() << std::endl;
      invalid_trasition.push_back(i);
    }
  }
  while (!invalid_trasition.empty())
  {
    size_t index = invalid_trasition.back();
    _cache.erase(_cache.begin() + index);
    invalid_trasition.pop_back();
  }
  get_impl()->save_transitions(_cache);
}

//==================================================
void TransitionTab::add_button_click()
{
  _cache.emplace_back("");
}

//==================================================
void TransitionTab::delete_button_click(size_t row_number)
{
  if (row_number > _cache.size()) return;
  _cache.erase(_cache.begin() + row_number);
}
