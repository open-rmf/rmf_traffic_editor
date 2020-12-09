#include <multi_select_combo_box.h>

#include <iostream>

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

void MultiSelectComboBox::line_update()
{
  size_t list_count = pListWidget->count();
  selectedText.clear();

  for (size_t i = 0; i < list_count; i++)
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
}