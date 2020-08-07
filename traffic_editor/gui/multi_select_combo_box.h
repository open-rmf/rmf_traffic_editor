#ifndef MULTI_SELECT_COMBO_BOX__H
#define MULTI_SELECT_COMBO_BOX__H

#include <QComboBox>
#include <QtWidgets>

#include <string>
#include <sstream>
#include <vector>
#include <set>

class MultiSelectComboBox : public QComboBox
{
public:
    template<typename ITEM_TYPE>
    MultiSelectComboBox(const std::vector<ITEM_TYPE>& selection_list) {
        selections.clear();
        check_result.clear();
        for (auto item : selection_list) {
            std::stringstream s;
            std::string temp;
            s << item;
            s >> temp;
            selections.emplace_back(temp);
            check_result.emplace_back(false);
        }

        build_list();
    }

    ~MultiSelectComboBox() {}

    std::vector<bool> getCheckResult();

private:
    void build_list();
    void box_checked(int state);
    void text_changed(const QString& text);

    std::vector<std::string> selections;
    std::vector<bool> check_result;
    QString selectedText;

    QListWidget* pListWidget;
    QLineEdit* pLineEdit;

};

#endif