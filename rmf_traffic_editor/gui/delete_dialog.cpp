/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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

#include "delete_dialog.h"
#include <QtWidgets>

DeleteDialog::DeleteDialog(
  QWidget* parent,
  const Building& building,
  int level_idx,
  int selected_vertex_idx)
: QDialog(parent)
{

  setWindowTitle("Warning");
  QVBoxLayout* warning_message_vbox_layout = new QVBoxLayout;

  // check if vertex is a lift cabin
  std::vector<std::string> lift_levels;
  const auto& level = building.levels[level_idx];
  const auto& v = level.vertices[selected_vertex_idx];
  auto it = v.params.find("lift_cabin");
  if ((it != v.params.end()))
  {
    auto lift = building.get_lift(v.lift_cabin());

    for (const auto& level_door : lift.level_doors)
    {
      lift_levels.push_back(level_door.first);
    }

    std::string s;
    for (uint i = 0; i < lift_levels.size(); i++)
    {
      s = s + lift_levels[i];
      if (i < lift_levels.size() - 1)
      {
        s = s + ", ";
      }
    }
    warning_message_vbox_layout->addWidget(new QLabel(
        QString("This item is a lift cabin and is used in %1.\n "
        "Are you sure you want to delete this item?")
        .arg(QString::fromStdString(s))));
  }
  else
  {
    std::string s;

    auto num_used_edges = level.edges_with_vertex(selected_vertex_idx).size();
    std::map<std::string, int> edge_types;
    for (const auto& edge : level.edges_with_vertex(selected_vertex_idx))
    {
      edge_types[edge.type_to_string()]++;
    }
    for (const auto& edge_type : edge_types)
    {
      s = s + " - " + std::to_string(edge_type.second) + " " + edge_type.first +
        " edge(s) \n";
    }

    auto num_used_polygons =
      level.polygons_with_vertex(selected_vertex_idx).size();
    std::map<std::string, int> polygon_types;
    for (const auto& polygon : level.polygons_with_vertex(selected_vertex_idx))
    {
      polygon_types[polygon.type_to_string()]++;
    }
    for (const auto& polygon_type : polygon_types)
    {
      s = s + " - " + std::to_string(polygon_type.second) + " " +
        polygon_type.first + " polygon(s) \n";
    }

    warning_message_vbox_layout->addWidget(new QLabel(
        QString("This item is linked to: \n%1"
        "Are you sure you want to delete this item?")
        .arg(QString::fromStdString(s))));

  }

  QHBoxLayout* bottom_buttons_layout = new QHBoxLayout;
  _ok_button = new QPushButton("OK", this);  // first button = [enter] button
  bottom_buttons_layout->addWidget(_ok_button);
  connect(
    _ok_button, &QAbstractButton::clicked,
    this, &DeleteDialog::ok_button_clicked);

  _cancel_button = new QPushButton("Cancel", this);
  bottom_buttons_layout->addWidget(_cancel_button);
  connect(
    _cancel_button, &QAbstractButton::clicked,
    this, &QDialog::reject);

  QVBoxLayout* vbox_layout = new QVBoxLayout;
  vbox_layout->addLayout(warning_message_vbox_layout);
  vbox_layout->addLayout(bottom_buttons_layout);

  setLayout(vbox_layout);
}

DeleteDialog::~DeleteDialog()
{
}

void DeleteDialog::ok_button_clicked()
{
  accept();
}