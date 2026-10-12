/*
 * Copyright (C) 2020-2021 Open Source Robotics Foundation
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

#ifndef ZONE_DIALOG_H
#define ZONE_DIALOG_H

#include <string>
#include <unordered_set>
#include <vector>

#include <QDialog>
#include <QObject>

#include "zone.h"
#include "building.h"

class QLineEdit;
class QLabel;
class QTableWidget;
class QComboBox;
class QCheckBox;

class ZoneDialog : public QDialog
{
  Q_OBJECT

public:
  ZoneDialog(Zone& zone, Building& building);
  ~ZoneDialog();

private:
  Zone& _zone;
  Building& _building;

  Zone _zone_copy;
  std::vector<QString> _level_names;

  QLineEdit* _name_line_edit;
  QComboBox* _level_combo_box;
  QLineEdit* _x_line_edit;
  QLineEdit* _y_line_edit;
  QLineEdit* _yaw_line_edit;
  QLineEdit* _width_line_edit;
  QLineEdit* _depth_line_edit;

  QCheckBox* _vertexcheckbox;
  QCheckBox* _lanecheckbox;

  QTableWidget* _internal_vertex_table;
  QTableWidget* _external_vertex_table;
  QGraphicsView* _zone_view;
  QGraphicsScene* _zone_scene;

  QPushButton* _ok_button, * _cancel_button;

  void update_in_vertex_table();
  void update_ex_vertex_table();
  void in_vertex_table_cell_changed(int row, int col);

  void update_zone_view();

  bool is_zone_name_unique(const std::string& name) const;

  /// True if no vertex in the given list, other than ignore_index, uses name.
  template<typename VertexT>
  bool is_vertex_name_unique(
    const std::vector<VertexT>& vertices,
    const std::string& name,
    int ignore_index = -1) const
  {
    for (std::size_t i = 0; i < vertices.size(); ++i)
    {
      if (static_cast<int>(i) == ignore_index)
        continue;

      if (vertices[i].name == name)
        return false;
    }
    return true;
  }

  bool has_duplicate_priority(const std::vector<InternalVertex>& vertices);

  bool confirm_warning(const QString& text);

private slots:
  void ok_button_clicked();
  void cancel_button_clicked();

signals:
  void redraw();
};

#endif
