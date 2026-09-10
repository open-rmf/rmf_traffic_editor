/*
 * Copyright (C) 2019-2020 Open Source Robotics Foundation
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

#include "zone_dialog.h"
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <QtWidgets>
using std::vector;

ZoneDialog::ZoneDialog(Zone& zone, Building& building)
: QDialog(),
  _zone(zone),
  _building(building)
{
  setWindowTitle("Zone Properties");
  for (const auto& level : building.levels)
    _level_names.push_back(QString::fromStdString(level.name));

  _zone_copy = _zone;

  QHBoxLayout* bottom_buttons_hbox = new QHBoxLayout;
  _ok_button = new QPushButton("OK", this);  // first button = [enter] button
  bottom_buttons_hbox->addWidget(_ok_button);
  connect(
    _ok_button, &QAbstractButton::clicked,
    this, &ZoneDialog::ok_button_clicked);

  _cancel_button = new QPushButton("Cancel", this);
  bottom_buttons_hbox->addWidget(_cancel_button);
  connect(
    _cancel_button,
    &QAbstractButton::clicked,
    this,
    &ZoneDialog::cancel_button_clicked);

  QHBoxLayout* name_hbox = new QHBoxLayout;
  name_hbox->addWidget(new QLabel("Name:"));
  _name_line_edit =
    new QLineEdit(QString::fromStdString(_zone.name), this);
  connect(
    _name_line_edit,
    &QLineEdit::textEdited,
    [this](const QString& text)
    {
      _zone.name = text.toStdString();
      update_zone_view();
      emit redraw();
    });
  _name_line_edit->setToolTip("Name of the zone. (must be unique)");
  name_hbox->addWidget(_name_line_edit);

  QHBoxLayout* level_hbox = new QHBoxLayout;
  level_hbox->addWidget(new QLabel("Zone level:"));
  _level_combo_box = new QComboBox;
  for (const QString& level_name : _level_names)
    _level_combo_box->addItem(level_name);
  _level_combo_box->setCurrentText(
    QString::fromStdString(_zone.level));
  connect(
    _level_combo_box,
    &QComboBox::currentTextChanged,
    [this](const QString& text)
    {
      _zone.level = text.toStdString();
      for (const auto& level : _building.levels)
      {
        if (level.name == _zone.level)
        {
          _zone.elevation = level.elevation;
          _zone.external_vertices.clear();
        }
      }
      update_ex_vertex_table();
      update_zone_view();
      emit redraw();
    });
  _level_combo_box->setToolTip("Level which the zone is in.");
  level_hbox->addWidget(_level_combo_box);

  QHBoxLayout* x_hbox = new QHBoxLayout;
  x_hbox->addWidget(new QLabel("X (pixel):"));
  _x_line_edit =
    new QLineEdit(QString::number(_zone.x), this);
  connect(
    _x_line_edit,
    &QLineEdit::textEdited,
    [this](const QString& text)
    {
      _zone.x = text.toDouble();
      emit redraw();
    });
  _x_line_edit->setToolTip(
    "This <X> refers to the global <X> of the zone");
  x_hbox->addWidget(_x_line_edit);

  QHBoxLayout* y_hbox = new QHBoxLayout;
  y_hbox->addWidget(new QLabel("Y (pixel):"));
  _y_line_edit =
    new QLineEdit(QString::number(_zone.y), this);
  connect(
    _y_line_edit,
    &QLineEdit::textEdited,
    [this](const QString& text)
    {
      _zone.y = text.toDouble();
      emit redraw();
    });
  _y_line_edit->setToolTip(
    "This <Y> refers to  the global <Y> of the zone");
  y_hbox->addWidget(_y_line_edit);

  QHBoxLayout* yaw_hbox = new QHBoxLayout;
  yaw_hbox->addWidget(new QLabel("Yaw (rad):"));
  _yaw_line_edit =
    new QLineEdit(QString::number(_zone.yaw), this);
  connect(
    _yaw_line_edit,
    &QLineEdit::textEdited,
    [this](const QString& text)
    {
      _zone.yaw = text.toDouble();
      update_zone_view();
      emit redraw();
    });
  _yaw_line_edit->setToolTip(
    "This <Yaw> refers to the global <Yaw> of the zone.\n The yaw is in radians.");
  yaw_hbox->addWidget(_yaw_line_edit);

  QHBoxLayout* width_hbox = new QHBoxLayout;
  width_hbox->addWidget(new QLabel("Zone width (m):"));
  _width_line_edit =
    new QLineEdit(QString::number(_zone.width), this);
  connect(
    _width_line_edit,
    &QLineEdit::textEdited,
    [this](const QString& text)
    {
      _zone.width = text.toDouble();
      update_zone_view();
      emit redraw();
    });
  _width_line_edit->setToolTip("Width of the zone.");
  width_hbox->addWidget(_width_line_edit);

  QHBoxLayout* depth_hbox = new QHBoxLayout;
  depth_hbox->addWidget(new QLabel("Zone depth (m):"));
  _depth_line_edit =
    new QLineEdit(QString::number(_zone.depth), this);
  connect(
    _depth_line_edit,
    &QLineEdit::textEdited,
    [this](const QString& text)
    {
      _zone.depth = text.toDouble();
      update_zone_view();
      emit redraw();
    });
  _depth_line_edit->setToolTip("Depth of the zone.");
  depth_hbox->addWidget(_depth_line_edit);

  QHBoxLayout* show_vertices_hbox = new QHBoxLayout;
  show_vertices_hbox->addWidget(new QLabel("Show vertices:"));
  _vertexcheckbox = new QCheckBox;
  _vertexcheckbox->setChecked(_zone.show_vertices);
  connect(
    _vertexcheckbox,
    &QAbstractButton::clicked,
    [this](bool box_checked)
    {
      _zone.show_vertices = box_checked;
      update_zone_view();
      emit redraw();
    });
  show_vertices_hbox->addWidget(_vertexcheckbox);

  QHBoxLayout* show_lanes_hbox = new QHBoxLayout;
  show_lanes_hbox->addWidget(new QLabel("Show lanes:"));
  _lanecheckbox = new QCheckBox;
  _lanecheckbox->setChecked(_zone.show_lanes);
  connect(
    _lanecheckbox,
    &QAbstractButton::clicked,
    [this](bool box_checked)
    {
      _zone.show_lanes = box_checked;
      emit redraw();
    });
  show_lanes_hbox->addWidget(_lanecheckbox);

  _internal_vertex_table = new QTableWidget;
  _internal_vertex_table->setMinimumSize(600, 200);
  _internal_vertex_table->verticalHeader()->setVisible(false);
  _internal_vertex_table->setColumnCount(6);
  _internal_vertex_table->setSizeAdjustPolicy(
    QAbstractScrollArea::AdjustToContents);

  _internal_vertex_table->setHorizontalHeaderItem(0,
    new QTableWidgetItem("Add/Remove"));
  _internal_vertex_table->horizontalHeader()->setSectionResizeMode(
    0, QHeaderView::Fixed);
  _internal_vertex_table->setColumnWidth(0, 100);

  _internal_vertex_table->setHorizontalHeaderItem(1,
    new QTableWidgetItem("Internal Vertex"));
  _internal_vertex_table->horizontalHeader()->setSectionResizeMode(
    1, QHeaderView::Fixed);
  _internal_vertex_table->setColumnWidth(1, 250);

  _internal_vertex_table->setHorizontalHeaderItem(2,
    new QTableWidgetItem("X (m)"));
  _internal_vertex_table->horizontalHeader()->setSectionResizeMode(
    2, QHeaderView::ResizeToContents);

  _internal_vertex_table->setHorizontalHeaderItem(3,
    new QTableWidgetItem("Y (m)"));
  _internal_vertex_table->horizontalHeader()->setSectionResizeMode(
    3, QHeaderView::ResizeToContents);

  _internal_vertex_table->setHorizontalHeaderItem(4,
    new QTableWidgetItem("Group"));
  _internal_vertex_table->horizontalHeader()->setSectionResizeMode(
    4, QHeaderView::Stretch);

  _internal_vertex_table->setHorizontalHeaderItem(5,
    new QTableWidgetItem("Priority"));
  _internal_vertex_table->horizontalHeader()->setSectionResizeMode(
    5, QHeaderView::ResizeToContents);

  _external_vertex_table = new QTableWidget;
  _external_vertex_table->setMinimumSize(600, 200);
  _external_vertex_table->verticalHeader()->setVisible(false);
  _external_vertex_table->setColumnCount(4);
  _external_vertex_table->setSizeAdjustPolicy(
    QAbstractScrollArea::AdjustToContents);

  _external_vertex_table->setHorizontalHeaderItem(0,
    new QTableWidgetItem("Add/Remove"));
  _external_vertex_table->horizontalHeader()->setSectionResizeMode(
    0, QHeaderView::Fixed);
  _external_vertex_table->setColumnWidth(0, 100);

  _external_vertex_table->setHorizontalHeaderItem(1,
    new QTableWidgetItem("External vertex"));
  _external_vertex_table->horizontalHeader()->setSectionResizeMode(
    1, QHeaderView::Stretch);

  _external_vertex_table->setHorizontalHeaderItem(2,
    new QTableWidgetItem("is entry"));
  _external_vertex_table->horizontalHeader()->setSectionResizeMode(
    2, QHeaderView::ResizeToContents);

  _external_vertex_table->setHorizontalHeaderItem(3,
    new QTableWidgetItem("is exit"));
  _external_vertex_table->horizontalHeader()->setSectionResizeMode(
    3, QHeaderView::ResizeToContents);

  QVBoxLayout* left_vbox = new QVBoxLayout;
  left_vbox->addLayout(name_hbox);
  left_vbox->addLayout(level_hbox);
  left_vbox->addLayout(x_hbox);
  left_vbox->addLayout(y_hbox);
  left_vbox->addLayout(yaw_hbox);
  left_vbox->addLayout(width_hbox);
  left_vbox->addLayout(depth_hbox);
  left_vbox->addWidget(_internal_vertex_table);
  left_vbox->addWidget(_external_vertex_table);

  QVBoxLayout* right_vbox = new QVBoxLayout;
  _zone_scene = new QGraphicsScene;
  _zone_view = new QGraphicsView;
  _zone_view->setScene(_zone_scene);
  _zone_view->setMinimumSize(400, 400);
  right_vbox->addWidget(_zone_view, 1);
  right_vbox->addLayout(show_vertices_hbox);
  right_vbox->addLayout(show_lanes_hbox);

  QHBoxLayout* top_hbox = new QHBoxLayout;
  top_hbox->addLayout(left_vbox, 1);
  top_hbox->addLayout(right_vbox, 1);

  QVBoxLayout* top_vbox = new QVBoxLayout;
  top_vbox->addLayout(top_hbox);
  // todo: some sort of separator (?)
  top_vbox->addLayout(bottom_buttons_hbox);

  setLayout(top_vbox);

  connect(
    _internal_vertex_table, &QTableWidget::cellChanged,
    this, &ZoneDialog::in_vertex_table_cell_changed);

  update_in_vertex_table();
  update_ex_vertex_table();
  update_zone_view();
  adjustSize();
  _ok_button->setFocus(Qt::OtherFocusReason);
  _ok_button->setDefault(true);
}

// ======================================================================================================================
ZoneDialog::~ZoneDialog()
{
}

// ======================================================================================================================
bool ZoneDialog::is_zone_name_unique(const std::string& name) const
{
  for (const auto& zone : _building.zones)
  {
    // skip comparing with itself (important when editing existing zone)
    if (&zone == &_zone)
      continue;

    if (zone.name == name)
      return false;
  }
  return true;
}

// ======================================================================================================================
bool ZoneDialog::has_duplicate_priority(
  const std::vector<InternalVertex>& vertices)
{
  std::unordered_set<uint> seen;
  for (const auto& v : vertices)
  {
    if (!seen.insert(v.priority).second)
      return true;
  }
  return false;
}

// ======================================================================================================================
bool ZoneDialog::confirm_warning(const QString& text)
{
  QDialog dialog(this);
  dialog.setWindowTitle("Warning");

  QHBoxLayout* msg_hbox = new QHBoxLayout;
  QLabel* icon_label = new QLabel;
  icon_label->setPixmap(
    dialog.style()->standardIcon(QStyle::SP_MessageBoxWarning).pixmap(48, 48));
  msg_hbox->addWidget(icon_label);
  msg_hbox->addWidget(new QLabel(text), 1);

  QPushButton* ok_button = new QPushButton("OK");
  QPushButton* cancel_button = new QPushButton("Cancel");
  QHBoxLayout* button_hbox = new QHBoxLayout;
  button_hbox->addStretch(1);
  button_hbox->addWidget(ok_button);
  button_hbox->addWidget(cancel_button);

  QVBoxLayout* vbox = new QVBoxLayout;
  vbox->addLayout(msg_hbox);
  vbox->addLayout(button_hbox);
  dialog.setLayout(vbox);

  connect(ok_button, &QAbstractButton::clicked, &dialog, &QDialog::accept);
  connect(cancel_button, &QAbstractButton::clicked, &dialog, &QDialog::reject);

  return dialog.exec() == QDialog::Accepted;
}

// ======================================================================================================================
void ZoneDialog::ok_button_clicked()
{
  if (_name_line_edit->text().isEmpty())
  {
    QMessageBox::critical(this, "Error", "Zone name is empty");
    return;
  }

  if (!is_zone_name_unique(_name_line_edit->text().toStdString()))
  {
    QMessageBox::critical(this, "Error", "Zone name must be unique");
    return;
  }

  bool has_entry = false;
  bool has_exit = false;
  for (std::size_t i = 0; i < _zone.external_vertices.size(); i++)
  {
    const ExternalVertex& ev = _zone.external_vertices[i];
    if (ev.name.empty())
    {
      QMessageBox::critical(this, "Error",
        "External vertex name cannot be empty.");
      return;
    }

    if (!is_vertex_name_unique(
        _zone.external_vertices, ev.name, i))
    {
      QMessageBox::critical(this, "Error",
        QString::fromStdString(
          "Duplicate external vertex [" + ev.name + "]."));
      return;
    }

    has_entry = has_entry || ev.is_entry_point;
    has_exit = has_exit || ev.is_exit_point;
  }

  if (_zone.internal_vertices.empty())
  {
    if (!confirm_warning(
        "This zone has no internal vertices, so a robot has nowhere to go "
        "inside it."))
      return;
  }

  for (std::size_t i = 0; i < _zone.internal_vertices.size(); i++)
  {
    const InternalVertex& iv = _zone.internal_vertices[i];
    if (iv.name.empty())
    {
      QMessageBox::critical(this, "Error",
        "Internal vertex name cannot be empty.");
      return;
    }

    if (iv.group.empty())
    {
      QMessageBox::critical(this, "Error",
        "Internal vertex group cannot be empty.");
      return;
    }

    for (const auto& level : _building.levels)
    {
      if (!is_vertex_name_unique(level.vertices, iv.name))
      {
        QMessageBox::critical(this, "Error",
          QString::fromStdString(
            "Vertex name [" + iv.name + "] already exists on level [" +
            level.name + "]. Internal vertex names must be unique."));
        return;
      }
    }

    for (const auto& zone : _building.zones)
    {
      if (&zone == &_zone)
        continue;

      if (!is_vertex_name_unique(zone.internal_vertices, iv.name))
      {
        QMessageBox::critical(this, "Error",
          QString::fromStdString(
            "Vertex name [" + iv.name + "] is already used by zone [" +
            zone.name + "]. Internal vertex names must be unique."));
        return;
      }
    }

    if (!is_vertex_name_unique(_zone.internal_vertices, iv.name, i))
    {
      QMessageBox::critical(this, "Error",
        QString::fromStdString(
          "Duplicate internal vertex [" + iv.name + "]."));
      return;
    }
  }

  if (has_duplicate_priority(_zone.internal_vertices))
  {
    if (!confirm_warning(
        "Duplicate internal vertex priority."))
      return;
  }

  if (!has_entry || !has_exit)
  {
    QStringList missing;
    if (!has_entry)
      missing.append("Zone has no entry point.");
    if (!has_exit)
      missing.append("Zone has no exit point.");

    if (!confirm_warning(
        missing.join("\n") +
        "\nRobots will not be able to enter or leave this zone."))
      return;
  }

  QStringList outside;
  const double half_w = _zone.width / 2.0;
  const double half_d = _zone.depth / 2.0;
  for (const auto& iv: _zone.internal_vertices)
  {
    if (std::abs(iv.x) > half_w || std::abs(iv.y) > half_d)
      outside.append(QString::fromStdString(iv.name));
  }

  if (!outside.isEmpty())
  {
    if (!confirm_warning(
        "These internal vertices lie outside the zone bounds:\n  " +
        outside.join("\n  ")))
      return;
  }

  update_zone_view();
  emit redraw();
  accept();
}

// ======================================================================================================================
void ZoneDialog::cancel_button_clicked()
{
  _zone = _zone_copy;
  update_zone_view();
  emit redraw();
  reject();
}

// ======================================================================================================================
void ZoneDialog::update_in_vertex_table()
{
  _internal_vertex_table->blockSignals(true);
  _internal_vertex_table->setRowCount(1 + _zone.internal_vertices.size());
  for (std::size_t i = 0; i < _zone.internal_vertices.size(); i++)
  {
    const InternalVertex& vertex = _zone.internal_vertices[i];  // save some typing

    // clear any lingering placeholder items (e.g. when a row was previously the Add row)
    _internal_vertex_table->setItem(i, 0, new QTableWidgetItem());
    _internal_vertex_table->setItem(i, 5, new QTableWidgetItem());

    _internal_vertex_table->setItem(
      i, 1, new QTableWidgetItem(QString::fromStdString(vertex.name)));
    _internal_vertex_table->setItem(
      i, 2, new QTableWidgetItem(QString::number(vertex.x)));
    _internal_vertex_table->setItem(
      i, 3, new QTableWidgetItem(QString::number(vertex.y)));
    _internal_vertex_table->setItem(
      i, 4, new QTableWidgetItem(QString::fromStdString(vertex.group)));

    QPushButton* delete_button = new QPushButton("Delete...", this);
    _internal_vertex_table->setCellWidget(i, 0, delete_button);
    connect(
      delete_button,
      &QAbstractButton::clicked,
      [this, i]()
      {
        _zone.internal_vertices.erase(_zone.internal_vertices.begin() + i);
        update_in_vertex_table();
        update_zone_view();
      });

    QComboBox* priority_box = new QComboBox;
    // include the vertex's own priority even when it exceeds the current
    // vertex count, so a stale value is shown rather than silently clamped
    const std::size_t priority_count = std::max(
      _zone.internal_vertices.size(),
      static_cast<std::size_t>(vertex.priority));
    for (std::size_t j = 0; j < priority_count; j++)
    {
      priority_box->addItem(QString::number(j+1));
    }
    priority_box->setCurrentText(QString::number(vertex.priority));
    connect(
      priority_box,
      &QComboBox::currentTextChanged,
      [this, i](const QString& text)
      {
        _zone.internal_vertices[i].priority = text.toInt();
        update_in_vertex_table();
        update_zone_view();
        emit redraw();
      });
    _internal_vertex_table->setCellWidget(i, 5, priority_box);
  }

  // we'll use the last row for the "Add" button
  const int last_row_idx = static_cast<int>(_zone.internal_vertices.size());
  QPushButton* add_button = new QPushButton("Add...", this);
  _internal_vertex_table->setCellWidget(last_row_idx, 0, add_button);
  for (int col = 1; col <= 5; col++)
  {
    _internal_vertex_table->setCellWidget(last_row_idx, col, nullptr);
    auto* placeholder = new QTableWidgetItem();
    placeholder->setFlags(Qt::NoItemFlags);
    placeholder->setBackground(QColor(180, 180, 180));
    _internal_vertex_table->setItem(last_row_idx, col, placeholder);
  }
  connect(
    add_button,
    &QAbstractButton::clicked,
    [this]()
    {
      InternalVertex vertex;
      vertex.name = "";
      vertex.priority = _zone.internal_vertices.size()+1;
      _zone.internal_vertices.push_back(vertex);
      update_in_vertex_table();
      update_zone_view();
    });
  _internal_vertex_table->blockSignals(false);
}

// ======================================================================================================================
void ZoneDialog::update_ex_vertex_table()
{
  _external_vertex_table->blockSignals(true);
  _external_vertex_table->setRowCount(1 + _zone.external_vertices.size());

  for (std::size_t i = 0; i < _zone.external_vertices.size(); i++)
  {
    const ExternalVertex& ev = _zone.external_vertices[i];  // save some typing

    // clear any lingering placeholder items (e.g. when a row was previously the Add row)
    for (int col = 0; col <= 3; col++)
      _external_vertex_table->setItem(i, col, new QTableWidgetItem());

    QPushButton* delete_button = new QPushButton("Delete...", this);
    _external_vertex_table->setCellWidget(i, 0, delete_button);
    connect(
      delete_button,
      &QAbstractButton::clicked,
      [this, i]()
      {
        _zone.external_vertices.erase(_zone.external_vertices.begin() + i);
        update_ex_vertex_table();
        update_zone_view();
      });

    QComboBox* vertex_combo_box = new QComboBox;
    vertex_combo_box->addItem("Select vertex");
    for (const auto& level : _building.levels)
    {
      if (level.name != _zone.level)
        continue;

      for (const auto& v : level.vertices)
      {
        if (!v.name.empty())
          vertex_combo_box->addItem(QString::fromStdString(v.name));
      }
    }
    vertex_combo_box->setCurrentText(
      QString::fromStdString(ev.name));
    connect(
      vertex_combo_box,
      &QComboBox::currentTextChanged,
      [this, i, vertex_combo_box](const QString& text)
      {
        if (text == "Select vertex")
        {
          _zone.external_vertices[i].name.clear();
          return;
        }
        _zone.external_vertices[i].name = text.toStdString();
      });
    _external_vertex_table->setCellWidget(i, 1, vertex_combo_box);

    QCheckBox* is_entry_checkbox = new QCheckBox;
    is_entry_checkbox->setChecked(ev.is_entry_point);
    connect(
      is_entry_checkbox,
      &QAbstractButton::clicked,
      [this, i](bool checked)
      {
        _zone.external_vertices[i].is_entry_point = checked;
      });
    QWidget* is_entry_container = new QWidget;
    QHBoxLayout* is_entry_layout = new QHBoxLayout(is_entry_container);
    is_entry_layout->addWidget(is_entry_checkbox);
    is_entry_layout->setAlignment(Qt::AlignCenter);
    is_entry_layout->setContentsMargins(0, 0, 0, 0);
    _external_vertex_table->setCellWidget(i, 2, is_entry_container);

    QCheckBox* is_exit_checkbox = new QCheckBox;
    is_exit_checkbox->setChecked(ev.is_exit_point);
    connect(
      is_exit_checkbox,
      &QAbstractButton::clicked,
      [this, i](bool checked)
      {
        _zone.external_vertices[i].is_exit_point = checked;
      });
    QWidget* is_exit_container = new QWidget;
    QHBoxLayout* is_exit_layout = new QHBoxLayout(is_exit_container);
    is_exit_layout->addWidget(is_exit_checkbox);
    is_exit_layout->setAlignment(Qt::AlignCenter);
    is_exit_layout->setContentsMargins(0, 0, 0, 0);
    _external_vertex_table->setCellWidget(i, 3, is_exit_container);
  }

  // we'll use the last row for the "Add" button
  const int last_row_idx = static_cast<int>(_zone.external_vertices.size());
  QPushButton* add_button = new QPushButton("Add...", this);
  _external_vertex_table->setCellWidget(last_row_idx, 0, add_button);
  for (int col = 1; col <= 3; col++)
  {
    _external_vertex_table->setCellWidget(last_row_idx, col, nullptr);
    auto* placeholder = new QTableWidgetItem();
    placeholder->setFlags(Qt::NoItemFlags);
    placeholder->setBackground(QColor(180, 180, 180));
    _external_vertex_table->setItem(last_row_idx, col, placeholder);
  }
  connect(
    add_button,
    &QAbstractButton::clicked,
    [this]()
    {
      ExternalVertex new_ev;
      _zone.external_vertices.push_back(new_ev);
      update_ex_vertex_table();
      update_zone_view();
    });
  _external_vertex_table->blockSignals(false);
}

// ======================================================================================================================
void ZoneDialog::in_vertex_table_cell_changed(int row, int col)
{
  if (row < 0 || row >= static_cast<int>(_zone.internal_vertices.size()))
    return;

  InternalVertex& vertex = _zone.internal_vertices[row];

  if (col == 1)  // name
  {
    const std::string name = _internal_vertex_table->
      item(row, col)->text().toStdString();
    vertex.name = name;
  }
  if (col == 2)  // x
    vertex.x = _internal_vertex_table->item(row, col)->text().toDouble();
  if (col == 3)  // y
    vertex.y = _internal_vertex_table->item(row, col)->text().toDouble();
  if (col == 4)  // group
    vertex.group = _internal_vertex_table->item(row, col)->text().toStdString();
  if (col == 5)  // priority
  {
    const uint priority = _internal_vertex_table->
      item(row, col)->text().toUInt();
    if (priority < 1)
    {
      QMessageBox::warning(this, "Invalid priority",
        "Priority must be a positive integer");
      _internal_vertex_table->blockSignals(true);
      _internal_vertex_table->item(row, col)->setText(
        QString::number(vertex.priority));
      _internal_vertex_table->blockSignals(false);
      return;
    }
    vertex.priority = priority;
  }

  update_in_vertex_table();
  update_zone_view();
  emit redraw();
}

// ======================================================================================================================
void ZoneDialog::update_zone_view()
{
  _zone_scene->clear();
  _zone.draw(_zone_scene, 0.01, std::string(), false);
}
