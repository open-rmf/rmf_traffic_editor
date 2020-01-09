/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef EDITOR_H
#define EDITOR_H

#include <map>
#include <string>
#include <vector>

#include <QGraphicsItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsPixmapItem>
#include <QGraphicsPolygonItem>
#include <QGraphicsScene>
#include <QMainWindow>
#include <QSettings>


class MapView;
class Level;
#include "./map.h"
#include "editor_model.h"

QT_BEGIN_NAMESPACE
class QAction;
class QMenu;
class QGraphicsView;
class QToolButton;
class QButtonGroup;
class QTableWidget;
class QTableWidgetItem;
class QLabel;
class QLineEdit;
class QListWidget;
class QMouseEvent;
class QHBoxLayout;
class QPushButton;
class QTabWidget;
QT_END_NAMESPACE


class Editor : public QMainWindow
{
  Q_OBJECT

public:
  Editor(QWidget *parent = nullptr);
  static Editor *get_instance();

  /// Load a project, replacing the current Map with a Map inflated from
  /// the YAML filename provided to this function.
  bool load_project(const QString &filename);

  /// Attempt to load the most recently saved project, just for convenience
  /// when starting the application since often we want to 'resume' editing.
  bool load_previous_project();

  enum {
    SELECT = 1,
    ADD_VERTEX,
    MOVE_VERTEX,
    ADD_LANE,
    ADD_WALL,
    ADD_MEAS,
    ADD_DOOR,
    ADD_MODEL,
    ROTATE_MODEL,
    MOVE_MODEL,
    ADD_FLOOR,
    EDIT_POLYGON,
    ADD_ZONE
  };

protected:
  void mousePressEvent(QMouseEvent *e);
  void mouseReleaseEvent(QMouseEvent *e);
  void mouseMoveEvent(QMouseEvent *e);
  void keyPressEvent(QKeyEvent *event);

private slots:
  void new_map();
  void open();
  void save();
  void about();

private:
  void edit_preferences();

  void level_add();
  void level_edit();
  void update_level_buttons();

  void zoom_fit();

  bool is_mouse_event_in_map(QMouseEvent *e, QPointF &p_scene);

  QToolBar *toolbar;
  QToolButton *create_tool_button(const int id);
  void tool_toggled(int id, bool checked);

/////////////////////////////
  static Editor *instance;  // there will only be one instance

  Map map;
  int level_idx;  // level that we are currently editing
  int clicked_idx;  // point most recently clicked
  int polygon_idx;  // currently selected polygon

  QButtonGroup *level_button_group;
  QHBoxLayout *level_button_hbox_layout;
  QGraphicsScene *scene;
  MapView *map_view;

  QAction *save_action;
  QAction *zoom_in_action, *zoom_out_action;
  QAction *zoom_normal_action, *zoom_fit_action;

  QString project_filename;

  const QString tool_id_to_string(const int id);
  QButtonGroup *tool_button_group;

  QTabWidget *right_tab_widget;

  QTableWidget *layers_table;
  void populate_layers_table();
  void layers_table_set_row(
      const int row_idx,
      const QString &label,
      const bool checked);
  void layer_edit_button_clicked(const std::string &label);
  void layer_add_button_clicked();

  QTableWidget *levels_table;
  void populate_levels_table();

  QTableWidget *property_editor;
  void update_property_editor();
  void populate_property_editor(const Edge &edge);
  void populate_property_editor(const Model &model);
  void populate_property_editor(const Vertex &vertex);
  void clear_property_editor();
  QTableWidgetItem *create_table_item(const QString &str, bool editable=false);
  void property_editor_cell_changed(int row, int column);
  void property_editor_set_row(
      const int row_idx,
      const QString &label,
      const QString &value,
      const bool editable = false);
  void property_editor_set_row(
      const int row_idx,
      const QString &label,
      const int &value,
      const bool editable = false);
  void property_editor_set_row(
      const int row_idx,
      const QString &label,
      const double &value,
      const int max_decimal_places = 3,
      const bool editable = false);
  QPushButton *add_param_button, *delete_param_button;
  void add_param_button_clicked();
  void delete_param_button_clicked();

  std::vector<EditorModel> editor_models;
  EditorModel *mouse_motion_editor_model = nullptr;
  void load_model_names();

  int get_polygon_idx(const double x, const double y);

  bool create_scene();
  void clear_selection();

  const static int ROTATION_INDICATOR_RADIUS = 50;
  QGraphicsLineItem *mouse_motion_line;
  QGraphicsEllipseItem *mouse_motion_ellipse;
  QGraphicsPixmapItem *mouse_motion_model;
  QGraphicsPolygonItem *mouse_motion_polygon;
  std::vector<int> mouse_motion_polygon_vertices;
  int mouse_motion_polygon_vertex_idx;

  int tool_id;

  void draw_mouse_motion_line_item(const double mouse_x, const double mouse_y);
  void remove_mouse_motion_item();
  void set_selected_line_item(QGraphicsLineItem *line_item);
  void set_selected_pixmap_item(QGraphicsPixmapItem *item);
  void set_selected_ellipse_item(QGraphicsEllipseItem *item);

  bool line_vertices_match(
      const QGraphicsLineItem *line_item,
      const Vertex &v1,
      const Vertex &v2,
      const double max_dist);

  void number_key_pressed(const int n);

  // mouse handlers for various tools
  enum MouseType {
    UNDEFINED = 0,
    PRESS = 1,
    RELEASE = 2,
    MOVE = 3
  };

  void mouse_event(const MouseType t, QMouseEvent *e);

  // helper function to avoid repeating lots of "add edge" code
  void mouse_add_edge(
      const MouseType t,
      QMouseEvent *e,
      const QPointF &p,
      const Edge::Type &edge_type);

  // helper function to avoid repeating lots of "add polygon" code
  void mouse_add_polygon(
      const MouseType t,
      QMouseEvent *e,
      const QPointF &p,
      const Polygon::Type &polygon_type);

  QGraphicsPixmapItem *get_closest_pixmap_item(const QPointF &p);
  double discretize_angle(const double &angle);

  void mouse_select(const MouseType t, QMouseEvent *e, const QPointF &p);
  void mouse_add_vertex(const MouseType t, QMouseEvent *e, const QPointF &p);
  void mouse_move_vertex(const MouseType t, QMouseEvent *e, const QPointF &p);
  void mouse_add_lane(const MouseType t, QMouseEvent *e, const QPointF &p);
  void mouse_add_wall(const MouseType t, QMouseEvent *e, const QPointF &p);
  void mouse_add_meas(const MouseType t, QMouseEvent *e, const QPointF &p);
  void mouse_add_door(const MouseType t, QMouseEvent *e, const QPointF &p);
  void mouse_add_model(const MouseType t, QMouseEvent *e, const QPointF &p);
  void mouse_rotate_model(const MouseType t, QMouseEvent *e, const QPointF &p);
  void mouse_move_model(const MouseType t, QMouseEvent *e, const QPointF &p);
  void mouse_add_floor(const MouseType t, QMouseEvent *e, const QPointF &p);
  void mouse_edit_polygon(const MouseType t, QMouseEvent *e, const QPointF &p);
  void mouse_add_zone(const MouseType t, QMouseEvent *e, const QPointF &p);

  QPointF previous_mouse_point;
};

#endif
