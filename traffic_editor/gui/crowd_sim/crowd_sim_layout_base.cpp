/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <QHeaderView>
#include <QListWidget>
#include <QTabWidget>
#include <QWidget>
#include <QPushButton>
#include <QStatusBar>

#include "crowd_sim_layout_base.h"

//========================================
CrowdSimLayoutBase::CrowdSimLayoutBase(CrowdSimImplPtr impl)
: QHBoxLayout(),
  _crowd_sim_impl(impl),
  _current_index(0),
  _to_delete(false)
{
  _right_panel = new QVBoxLayout();
}

//======================================
void CrowdSimLayoutBase::initialise()
{
  blockSignals(true);
  QVBoxLayout* leftPanel = new QVBoxLayout();
  _list_widget = new QListWidget();
  list_item_in_cache();
  _list_widget->setCurrentItem(_list_widget->item(_current_index));
  connect(
    _list_widget,
    &QListWidget::currentRowChanged,
    [this](int row)
    {
      if (!_to_delete) // don't do anything if we delete
      {
        save(); // save first before switching
        _current_index = row;
        update();
      }
    });
  leftPanel->addWidget(_list_widget);
  QStatusBar* statusMenu = new QStatusBar();
  QPushButton* add = new QPushButton("+");
  QPushButton* minus = new QPushButton("-");
  connect(add, &QAbstractButton::clicked, this,
    &CrowdSimLayoutBase::add_button_click);
  connect(minus, &QAbstractButton::clicked, this,
    &CrowdSimLayoutBase::delete_button_click);
  statusMenu->addWidget(add);
  statusMenu->addWidget(minus);
  leftPanel->addWidget(statusMenu);
  _list_widget->setMaximumWidth(300);
  statusMenu->setMaximumWidth(300);
  this->addLayout(leftPanel);
  initialise_item_detail();
  this->addLayout(_right_panel);
  blockSignals(false);
}

//======================================
void CrowdSimLayoutBase::update()
{
  blockSignals(true);
  update_item_detail();
  blockSignals(false);
}