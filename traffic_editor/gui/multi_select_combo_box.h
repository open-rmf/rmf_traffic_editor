#ifndef MULTI_SELECT_COMBO_BOX__H
#define MULTI_SELECT_COMBO_BOX__H

#include <QComboBox>
#include <QtWidgets>

#include <string>
#include <sstream>
#include <vector>
#include <set>
#include "callbacks.h"

class MultiSelectComboBox : public QComboBox
{
public:
  template<typename ITEM_TYPE, typename T>
  MultiSelectComboBox(const std::vector<ITEM_TYPE>& selection_list, std::vector<std::pair<void (T::*)(), T*>> callbacks)
  {
    _callbacks = std::shared_ptr<void>(new Callbacks<T>(callbacks));
    selections.clear();
    for (auto item : selection_list)
    {
      selections.emplace_back(type_to_string(item), false);
    }
    build_list<T>();
  }

  template<typename ITEM_TYPE, typename T>
  MultiSelectComboBox(const std::set<ITEM_TYPE>& selection_list, std::vector<std::pair<void (T::*)(), T*>> callbacks)
  {
    _callbacks = std::shared_ptr<void>(new Callbacks<T>(callbacks));
    selections.clear();
    for (auto item : selection_list)
    {
      selections.emplace_back(type_to_string(item), false);
    }
    build_list<T>();
  }

  ~MultiSelectComboBox() {}

  template<typename ITEM_TYPE>
  void showCheckedItem(const std::set<ITEM_TYPE>& checked_list)
  {
    blockSignals(true);
    std::set<std::string> checked_item;
    for (auto item : checked_list)
    {
      checked_item.insert(type_to_string(item) );
    }

    auto list_count = pListWidget->count();
    for (auto i = 0; i < list_count; i++)
    {
      QListWidgetItem* pItem = pListWidget->item(i);
      QCheckBox* pCheckBox =
        static_cast<QCheckBox*>(pListWidget->itemWidget(pItem));
      auto checkbox_text = pCheckBox->text();
      if (checked_item.find(checkbox_text.toStdString() ) ==
        checked_item.end() )
      {
        // original goal area in project might be deleted, so we won't update the old version
        continue;
      }
      pCheckBox->setCheckState(Qt::Checked);
    }
    line_update();
    blockSignals(false);
  }

  template<typename ITEM_TYPE>
  std::string type_to_string(ITEM_TYPE& input)
  {
    std::stringstream s;
    std::string temp;
    s << input;
    s >> temp;
    return temp;
  }

  void line_update();
  std::vector<std::string> getCheckResult();
private:
  template<typename T>
  void build_list()
  {
    pListWidget = new QListWidget(this);
    pLineEdit = new QLineEdit(this);
    pLineEdit->setReadOnly(true);

    connect(
      pLineEdit,
      &QLineEdit::textChanged,
      [this](const QString& text)
      {
        text_changed(text);
      }
    );

    for (size_t i = 0; i < selections.size(); i++)
    {
      QListWidgetItem* pListItem = new QListWidgetItem(pListWidget);
      pListWidget->addItem(pListItem);
      QCheckBox* pCheckBox =
        new QCheckBox(QString::fromStdString(selections[i].first));
      pListWidget->setItemWidget(pListItem, pCheckBox);

      connect(
        pCheckBox,
        &QAbstractButton::clicked,
        [&](int state)
        {
          box_checked<T>(state);
        }
      );
    }

    this->setModel(pListWidget->model());
    this->setView(pListWidget);
    this->setLineEdit(pLineEdit);
  }

  template<typename T>
  void box_checked(int state)
  {
    blockSignals(true);
    auto callbacks_ptr = std::static_pointer_cast<Callbacks<T>>(_callbacks);
    line_update();
    callbacks_ptr->initiate();
    blockSignals(false);
  }

  void text_changed(const QString& text);

  std::shared_ptr<void> _callbacks;
  std::vector<std::pair<std::string, bool>> selections;
  QString selectedText;

  QListWidget* pListWidget;
  QLineEdit* pLineEdit;

};

#endif