#include <crowd_sim_dialog.h>

#include <QtWidgets>

#include <iostream>

// CrowdSimDialog::CrowdSimDialog(CrowdSimImplPtr implPtr) 
// : crowd_sim_impl(implPtr)
// {
//     // ok_button = new QPushButton("OK", this);
//     // cancel_button = new QPushButton("Cancel", this);
//     // QHBoxLayout* bottom_buttons_hbox = new QHBoxLayout;
//     // bottom_buttons_hbox->addWidget(cancel_button);
//     // bottom_buttons_hbox->addWidget(ok_button);


//     // QVBoxLayout top_vobx;
//     // top_vbox.addLayout(bottom_guttons_hbox);
//     // setLayout(top_vbox);
// }


StatesDialog::StatesDialog(CrowdSimImplPtr implPtr) 
    : crowd_sim_impl(implPtr) 
{
    setWindowTitle("States");

    QHBoxLayout* table_box = new QHBoxLayout;
    table_box->addWidget(states_tab.get());

    QVBoxLayout* top_vbox = new QVBoxLayout;
    top_vbox->addLayout(table_box);
    setLayout(top_vbox);

}

// void StatesDialog::update(){

//     blockSignals(true);
//     size_t states_count = crowd_sim_impl->states.size();
//     setRowCount(1 + crowd_sim_impl->states.size());

//     for (size_t i = 0; i < states_count; i++){

//     }

//     const int last_row_id = static_cast<int>(states_count);
    
//     QPushButton add_button("Add...", this);
//     setCellWidget(last_row_id, 4, &add_button);
//     connect(
//         &add_button,
//         &QAbstractButton::clicked,
//         [this](){
//             std::cout << "Add button clicked" << std::endl;
//         }
//     );

//     blockSignals(false);
// }

// void StatesDialog::ok_button_clicked(){
    
// }

StatesTab::StatesTab() : TableList(5)
{
    const QStringList labels =
        { "Name", "Is Final", "Navmesh File Name", "Goal Set Id", ""};
    setHorizontalHeaderLabels(labels);
}

void StatesTab::update(CrowdSimImplPtr implPtr)
{
    blockSignals(true);

    


    blockSignals(false);
}

