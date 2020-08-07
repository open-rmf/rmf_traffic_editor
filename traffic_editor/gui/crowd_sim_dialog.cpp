#include <crowd_sim_dialog.h>
#include <multi_select_combo_box.h>

#include <iostream>
#include <sstream>
#include <string>

CrowdSimDialog::CrowdSimDialog(CrowdSimImplPtr implPtr) 
: crowd_sim_impl(implPtr)
{
    ok_button = new QPushButton("OK", this);
    cancel_button = new QPushButton("Cancel", this);
    bottom_buttons_hbox = new QHBoxLayout;
    bottom_buttons_hbox->addWidget(cancel_button);
    bottom_buttons_hbox->addWidget(ok_button);
    connect(
        ok_button,
        &QAbstractButton::clicked,
        [this](){
            this->ok_button_clicked();
        }
    );
    connect(
        cancel_button,
        &QAbstractButton::clicked,
        [this](){
            this->cancel_button_clicked();
        }
    );

    top_vbox = new QVBoxLayout(this);

}

void CrowdSimDialog::ok_button_clicked() {
    accept();
}

void CrowdSimDialog::cancel_button_clicked() {
    reject();
}

//==========================================================================
StatesDialog::StatesDialog(CrowdSimImplPtr implPtr) 
    : CrowdSimDialog(implPtr) 
{
    states_tab = std::make_shared<StatesTab>(crowd_sim_impl);
    states_tab->update();

    setWindowTitle("States");

    QHBoxLayout* table_box = new QHBoxLayout;
    table_box->addWidget(states_tab.get());

    top_vbox->addLayout(table_box);
    top_vbox->addLayout(bottom_buttons_hbox);

}

void StatesDialog::ok_button_clicked() {
    states_tab->save();
    accept();
}

StatesTab::StatesTab(CrowdSimImplPtr crowd_sim_impl) 
    : TableList(5), implPtr(crowd_sim_impl)
{
    const QStringList labels =
        { "Name", "Is Final", "Navmesh File Name", "Goal Set Id", ""};
    setHorizontalHeaderLabels(labels);
    setMinimumSize(800, 400);
}

void StatesTab::update()
{
    auto states_number = implPtr->states.size();    
    setRowCount(1 + states_number);
    clearContents();

    list_states_in_impl();

    QPushButton* add_button = new QPushButton("Add...", this);
    for (auto i = 0; i < 4; i++) {
        setItem(states_number, i, new QTableWidgetItem(QString::fromStdString("")));
    }
    setCellWidget(states_number, 4, add_button);
    connect(
        add_button,
        &QAbstractButton::clicked,
        [this](){
            add_button_clicked();
            update();
        }
    );
}

void StatesTab::list_states_in_impl() {

    auto states_number = implPtr->states.size();

    for (size_t i = 0; i < states_number; i++) {
        auto& current_state = implPtr->states.at(i);
        setItem(i, 0, new QTableWidgetItem(QString::fromStdString(current_state.getName()) ) );

        QComboBox* final_state_combo = new QComboBox;
        list_final_states_in_combo(final_state_combo, current_state.getFinalState());
        setCellWidget(i, 1, final_state_combo );
        connect(
            final_state_combo,
            &QComboBox::currentTextChanged,
            [&](const QString& text){
                current_state.setFinalState(text.toStdString() == "1");
            }
        );

        setItem(i, 2, new QTableWidgetItem(QString::fromStdString(current_state.getNavmeshFileName() )));

        QComboBox* goal_set_combo = new QComboBox;
        list_goal_sets_in_combo(goal_set_combo, current_state.getGoalSetId());
        setCellWidget(i, 3, goal_set_combo);
        connect(
            goal_set_combo,
            &QComboBox::currentTextChanged,
            [&](const QString& text){
                size_t goal_set_id;
                std::istringstream(text.toStdString()) >> goal_set_id;
                current_state.setGoalSetId(goal_set_id);
            }
        );
        
        QPushButton* delete_button = new QPushButton("Delete", this);
        setCellWidget(i, 4, delete_button);
        connect(
            delete_button,
            &QAbstractButton::clicked,
            [&, i](){
                implPtr->states.erase(implPtr->states.begin() + i);
                update();
            }
        );
    }

}

void StatesTab::list_goal_sets_in_combo(QComboBox* comboBox, size_t current_goal_set_id) {
    for (auto goal_set : implPtr->goal_sets) {
        comboBox->addItem(QString::number(static_cast<int>(goal_set.getGoalSetId())));
    }
    auto index = comboBox->findText(QString::number(static_cast<int>(current_goal_set_id) ));
    if (index >= 0) {
        comboBox->setCurrentIndex(index);
    }
}

void StatesTab::list_final_states_in_combo(QComboBox* comboBox, bool current_state) {
    comboBox->addItem("1");
    comboBox->addItem("0");
    if (!current_state) {
        comboBox->setCurrentIndex(1);
    } else {
        comboBox->setCurrentIndex(0);
    }
}

void StatesTab::add_button_clicked() {
    implPtr->states.emplace_back("state" + std::to_string( save() ));
}

int StatesTab::save() {
    auto rows_count = rowCount();
    for (auto i = 0; i < rows_count - 1; i++) {
        auto& current_state = implPtr->states.at(i);
        auto name_item = item(i, 0)->text().toStdString();
        current_state.setName(name_item);
    }
    return rows_count;
}

//==============================================================================

GoalSetTab::GoalSetTab(CrowdSimImplPtr crowd_sim_impl) 
    : TableList(3), implPtr(crowd_sim_impl) 
{
    const QStringList labels =
        { "Id", "Goal Area", ""};
    setHorizontalHeaderLabels(labels);
    resizeColumnsToContents();
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setMinimumSize(800, 400);
}

void GoalSetTab::update() {

    auto goal_set_number = implPtr->goal_sets.size();
    setRowCount(1 + goal_set_number);
    clearContents();
    list_goal_set_in_impl();

    QPushButton* add_button = new QPushButton("Add...", this);
    for (auto i = 0; i < 2; i++) {
        setItem(goal_set_number, i, new QTableWidgetItem(QString::fromStdString("")));
    }
    setCellWidget(goal_set_number, 2, add_button);
    connect(
        add_button,
        &QAbstractButton::clicked,
        [&](){
            add_button_clicked();
            update();
        }
    );

}

void GoalSetTab::list_goal_set_in_impl() {
    auto goal_set_number = implPtr->goal_sets.size();
    for (auto i = 0; i < goal_set_number; i++) {
        auto& goal_set = implPtr->goal_sets[i];
        setItem(i, 0, new QTableWidgetItem(QString::number(static_cast<int>(goal_set.getGoalSetId()))));

        MultiSelectComboBox* multi_combo_box = new MultiSelectComboBox(implPtr->getGoalAreas());
        setCellWidget(i, 1, multi_combo_box);
    }
}

int GoalSetTab::save() {
    auto row_count = rowCount();
    for (auto i = 0; i < row_count - 1; i++) {
        implPtr->goal_sets.clear();

        QTableWidgetItem* pItem_setid = item(i, 0);
        bool OK_status;
        auto set_id = pItem_setid->text().toInt(&OK_status);
        if(!OK_status) {
            std::cout << "Invalid goal set id in row " << i << " with [" << pItem_setid->text().toStdString() << "]";
            return -1; // should try exception?
        }

        implPtr->goal_sets.emplace_back(static_cast<size_t>(set_id));
        auto goal_set_iterator = implPtr->goal_sets.back();
        
        auto pItem_areas = static_cast<MultiSelectComboBox*>(cellWidget(i, 1));
        for(auto item : pItem_areas->getCheckResult() ) {
            goal_set_iterator.addGoalArea(item);
        }

    }
    return row_count;
}

void GoalSetTab::add_button_clicked() {
    auto row_count = save();
    size_t new_goal_set_id = 0;
    if(implPtr->goal_sets.size() != 0) {
        new_goal_set_id = implPtr->goal_sets.back().getGoalSetId() + 1;
    }
    implPtr->goal_sets.emplace_back(new_goal_set_id);
}


GoalSetDialog::GoalSetDialog(CrowdSimImplPtr crowd_sim_impl)
    : CrowdSimDialog(crowd_sim_impl)
{
    goal_set_tab = std::make_shared<GoalSetTab>(crowd_sim_impl);
    goal_set_tab->update();

    setWindowTitle("Goal Sets");

    QHBoxLayout* table_box = new QHBoxLayout;
    table_box->addWidget(goal_set_tab.get());

    top_vbox->addLayout(table_box);
    top_vbox->addLayout(bottom_buttons_hbox);
}

void GoalSetDialog::ok_button_clicked() {
    accept();
}
