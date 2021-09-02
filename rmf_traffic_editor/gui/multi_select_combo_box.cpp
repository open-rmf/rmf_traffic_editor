#include <multi_select_combo_box.h>

#include <iostream>

void MultiSelectComboBox::build_list()
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

  for (std::size_t i = 0; i < selections.size(); i++)
  {
    QListWidgetItem* pListItem = new QListWidgetItem(pListWidget);
    pListWidget->addItem(pListItem);
    QCheckBox* pCheckBox =
      new QCheckBox(QString::fromStdString(selections[i].first));
    pListWidget->setItemWidget(pListItem, pCheckBox);

    connect(
      pCheckBox,
      &QCheckBox::stateChanged,
      [this](int state)
      {
        box_checked(state);
      }
    );
  }

  this->setModel(pListWidget->model());
  this->setView(pListWidget);
  this->setLineEdit(pLineEdit);
}


void MultiSelectComboBox::box_checked(int /*state*/)
{
  blockSignals(true);
  std::size_t list_count = pListWidget->count();
  selectedText.clear();

  for (std::size_t i = 0; i < list_count; i++)
  {
    QListWidgetItem* pItem = pListWidget->item(i);
    QCheckBox* pCheckBox = static_cast<QCheckBox*>(pListWidget->itemWidget(
        pItem));
    if (pCheckBox->isChecked())
    {
      QString checkbox_text = pCheckBox->text();
      selectedText.append(checkbox_text).append(";");
      selections[i].second = true;
    }
    else
    {
      selections[i].second = false;
    }
  }

  if (selectedText.isEmpty())
  {
    pLineEdit->clear();
    return;
  }
  pLineEdit->setText(selectedText);
  pLineEdit->setToolTip(selectedText);
  blockSignals(false);
}

void MultiSelectComboBox::text_changed(const QString& text)
{
  pLineEdit->setText(text);
}

std::vector<std::string> MultiSelectComboBox::getCheckResult()
{
  std::vector<std::string> result;
  for (auto item : selections)
  {
    if (item.second)
    {
      result.emplace_back(item.first);
    }
  }
  return result;
}
