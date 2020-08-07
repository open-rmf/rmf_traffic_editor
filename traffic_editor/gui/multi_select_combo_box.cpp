#include <multi_select_combo_box.h>

#include <iostream>

// MultiSelectComboBox::MultiSelectComboBox(const std::vector<std::string>& selection_list)
// {
//     selections.clear();
//     check_result.clear();
//     for (size_t i = 0; i < selection_list.size(); i++) {
//         selections.emplace_back(selection_list[i]);
//         check_result.emplace_back(false);
//     }

//     build_list();
// }

void MultiSelectComboBox::build_list() {
    pListWidget = new QListWidget(this);
    pLineEdit = new QLineEdit(this);
    pLineEdit->setReadOnly(true);

    connect(
        pLineEdit,
        &QLineEdit::textChanged,
        [this](const QString& text){
            text_changed(text);
        }
    );

    for (size_t i = 0; i < selections.size(); i++) {
        QListWidgetItem* pListItem = new QListWidgetItem(pListWidget);
        pListWidget->addItem(pListItem);
        QCheckBox* pCheckBox = new QCheckBox(QString::fromStdString(selections[i]));
        pListWidget->setItemWidget(pListItem, pCheckBox);

        connect(
            pCheckBox,
            &QCheckBox::stateChanged,
            [this](int state){
                box_checked(state);
            }
        );
    }

    this->setModel(pListWidget->model());
    this->setView(pListWidget);
    this->setLineEdit(pLineEdit);
}


void MultiSelectComboBox::box_checked(int state) {
    blockSignals(true);
    size_t list_count = pListWidget->count();
    selectedText.clear();

    for (size_t i = 0; i < list_count; i++) {
        QListWidgetItem* pItem = pListWidget->item(i);
        QCheckBox* pCheckBox = static_cast<QCheckBox*>(pListWidget->itemWidget(pItem));
        if(pCheckBox->isChecked()) {
            QString checkbox_text = pCheckBox->text();
            selectedText.append(checkbox_text).append(";");
            check_result[i] = true;
        }
    }

    if (selectedText.isEmpty()) {
        pLineEdit->clear();
        return;
    }
    pLineEdit->setText(selectedText);
    pLineEdit->setToolTip(selectedText);
    blockSignals(false);
}

void MultiSelectComboBox::text_changed(const QString& text) {
    pLineEdit->setText(text);
}

std::vector<bool> MultiSelectComboBox::getCheckResult() {
    return check_result;
}