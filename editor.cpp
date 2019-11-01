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

#include <algorithm>
#include <cmath>
#include <string>

#include <QtWidgets>

#include <QInputDialog>
#include <QLabel>
#include <QListWidget>
#include <QToolBar>

#include <yaml-cpp/yaml.h>

#include "editor.h"
#include "level_dialog.h"
#include "map_view.h"


Editor *Editor::instance = nullptr;

Editor::Editor(QWidget *parent)
: QMainWindow(parent),
  level_idx(0),
  clicked_idx(-1),
  polygon_idx(-1),
  mouse_motion_line(nullptr),
  mouse_motion_ellipse(nullptr),
  mouse_motion_model(nullptr),
  mouse_motion_polygon(nullptr),
  mouse_motion_polygon_vertex_idx(-1),
  polygon_brush(QColor::fromRgbF(1.0, 1.0, 0.5, 0.5)),
  selected_polygon_brush(QColor::fromRgbF(1.0, 0.0, 0.0, 0.5)),
  tool_id(SELECT)
{
  instance = this;

  scene = new QGraphicsScene(this);

  map_view = new MapView(this);
  map_view->setScene(scene);

  level_button_group = new QButtonGroup(this);
  connect(
      level_button_group,
      QOverload<int, bool>::of(&QButtonGroup::buttonToggled),
      this, &Editor::level_button_toggled);

  level_button_hbox_layout = new QHBoxLayout;

  QVBoxLayout *map_layout = new QVBoxLayout;
  map_layout->addLayout(level_button_hbox_layout);
  map_layout->addWidget(map_view);

  property_editor = new QTableWidget();
  property_editor->setStyleSheet("QTableWidget { background-color: #e0e0e0; color: black; gridline-color: #606060; } QLineEdit { background:white; }");
  property_editor->setMinimumSize(300, 200);
  //property_editor->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Ignored);
  property_editor->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::MinimumExpanding);
  property_editor->setColumnCount(2);
  property_editor->horizontalHeader()->setVisible(false);
  property_editor->verticalHeader()->setVisible(false);
  property_editor->horizontalHeader()->setSectionResizeMode(
      0, QHeaderView::ResizeToContents);
  property_editor->horizontalHeader()->setSectionResizeMode(
      1, QHeaderView::Stretch);
  property_editor->verticalHeader()->setSectionResizeMode(
      QHeaderView::ResizeToContents);
  property_editor->setAutoFillBackground(true);
  connect(
      property_editor, &QTableWidget::cellChanged,
      this, &Editor::property_editor_cell_changed);

  QHBoxLayout *hbox_layout = new QHBoxLayout;
  hbox_layout->addLayout(map_layout, 1);
  hbox_layout->addWidget(property_editor);

  QWidget *w = new QWidget();
  w->setMouseTracking(true);
  setMouseTracking(true);
  w->setLayout(hbox_layout);
  w->setStyleSheet("background-color: #404040" );
  setCentralWidget(w);

  // FILE MENU
  QMenu *file_menu = menuBar()->addMenu("&File");

  QAction *new_action =
      file_menu->addAction("&New Project...", this, &Editor::new_map);
  new_action->setShortcut(QKeySequence::New);

  QAction *open_action =
      file_menu->addAction("&Open Project...", this, &Editor::open);
  open_action->setShortcut(QKeySequence::Open);

  save_action =
      file_menu->addAction("&Save Project", this, &Editor::save);
  save_action->setShortcut(tr("Ctrl+S"));

  file_menu->addSeparator();

  QAction *exit_action = file_menu->addAction("E&xit", this, &QWidget::close);
  exit_action->setShortcut(tr("Ctrl+Q"));

  // LEVEL MENU
  QMenu *level_menu = menuBar()->addMenu("&Level");
  level_menu->addAction("&Add...", this, &Editor::level_add);
  level_menu->addSeparator();
  level_menu->addAction("&Edit...", this, &Editor::level_edit);

  // VIEW MENU
  QMenu *view_menu = menuBar()->addMenu("&View");

  zoom_in_action = view_menu->addAction("Zoom &In", this, &Editor::zoom_in);
  zoom_in_action->setShortcut(QKeySequence::ZoomIn);
  //zoom_in_action->setEnabled(false);

  zoom_out_action =
      view_menu->addAction("Zoom &Out", this, &Editor::zoom_out);
  zoom_out_action->setShortcut(QKeySequence::ZoomOut);
  //zoom_out_action->setEnabled(false);

  zoom_normal_action =
      view_menu->addAction("&Normal Size", this, &Editor::zoom_normal);
  //zoom_normal_action->setEnabled(false);

  view_menu->addSeparator();

  zoom_fit_action =
      view_menu->addAction("&Fit to Window", this, &Editor::zoom_fit);
  zoom_fit_action->setEnabled(false);
  zoom_fit_action->setCheckable(true);
  zoom_fit_action->setShortcut(tr("Ctrl+F"));

  // HELP MENU
  QMenu *help_menu = menuBar()->addMenu("&Help");

  help_menu->addAction("&About", this, &Editor::about);
  help_menu->addAction("About &Qt", &QApplication::aboutQt);

  // TOOLBAR
  toolbar = new QToolBar();
  tool_button_group = new QButtonGroup(this);
  tool_button_group->setExclusive(true);

  create_tool_button(SELECT);
  create_tool_button(MOVE_VERTEX);
  create_tool_button(ADD_VERTEX);
  create_tool_button(ADD_LANE);
  create_tool_button(ADD_WALL);
  create_tool_button(ADD_MEAS);
  create_tool_button(ADD_MODEL);
  create_tool_button(MOVE_MODEL);
  create_tool_button(ROTATE_MODEL);
  create_tool_button(ADD_FLOOR);
  create_tool_button(EDIT_POLYGON);
  create_tool_button(ADD_ZONE);

  toolbar->addSeparator();
  model_name_line_edit = new QLineEdit("", this);
  toolbar->addWidget(model_name_line_edit);
  connect(
      model_name_line_edit,
      &QLineEdit::textEdited,
      this,
      &Editor::model_name_line_edited);

  model_name_list_widget = new QListWidget;
  populate_model_name_list_widget();
  toolbar->addWidget(model_name_list_widget);
  connect(
      model_name_list_widget,
      &QListWidget::currentRowChanged,
      this,
      &Editor::model_name_list_widget_changed);

  model_preview_label = new QLabel("preview");
  model_preview_label->setSizePolicy(
      QSizePolicy::Expanding,
      QSizePolicy::MinimumExpanding);
  //model_preview_label->setScaledContents(true);
  toolbar->addWidget(model_preview_label);

  connect(
      tool_button_group,
      QOverload<int, bool>::of(&QButtonGroup::buttonToggled),
      this, &Editor::tool_toggled);

  toolbar->setStyleSheet("QToolBar {background-color: #404040; border: none; spacing: 5px} QToolButton {background-color: #c0c0c0; color: blue; border: 1px solid black;} QToolButton:checked {background-color: #808080; color: red; border: 1px solid black;}");
  addToolBar(Qt::LeftToolBarArea, toolbar);

  // SET SIZE
  resize(QGuiApplication::primaryScreen()->availableSize() / 2);
  map_view->adjustSize();

  // default tool is the "select" tool
  tool_button_group->button(SELECT)->click();
}

void Editor::populate_model_name_list_widget()
{
  // This function may throw exceptions. Caller should be ready for them!
  YAML::Node y;
  std::string filename("thumbnails/model_list.yaml");
  try {
    y = YAML::LoadFile(filename);
  }
  catch (const std::exception &e) {
    qWarning("couldn't parse %s: %s", filename.c_str(), e.what());
    return;
  }
  qInfo("parsed %s successfully", filename.c_str());

  const double model_meters_per_pixel = y["meters_per_pixel"].as<double>();
  const YAML::Node ym = y["models"];
  for (YAML::const_iterator it = ym.begin(); it != ym.end(); ++it) {
    std::string model_name = it->as<std::string>();
    model_name_list_widget->addItem(model_name.c_str());
    models.push_back(EditorModel(model_name, model_meters_per_pixel));
  }
}

QToolButton *Editor::create_tool_button(const int id)
{
  QToolButton *b = new QToolButton(toolbar);
  b->setText(tool_id_to_string(id));
  b->setCheckable(true);
  b->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Maximum);
  toolbar->addWidget(b);
  tool_button_group->addButton(b, id);
  return b;
}

Editor *Editor::get_instance()
{
  return instance;
}

bool Editor::load_project(const QString &filename)
{
  const std::string filename_std_string = filename.toStdString();
  try {
    map.load_yaml(filename_std_string);
    qInfo("parsed %s successfully", qUtf8Printable(filename));
  }
  catch (const std::exception &e) {
    qWarning("couldn't parse %s: %s",
        qUtf8Printable(filename),
        e.what());
    return false;
  }

  level_idx = 0;
  update_level_buttons();

  if (!map.levels.empty()) {
    level_button_group->button(level_idx)->setChecked(true);
    const Level &level = map.levels[level_idx];
    scene->setSceneRect(
        QRectF(0, 0, level.drawing_width, level.drawing_height));
  }

  create_scene();
  project_filename = filename;
  map_view->zoom_fit(map, level_idx);

  return true;
}

void Editor::update_level_buttons()
{
  // todo: remove all existing level selection buttons!
  QList<QAbstractButton *> buttons = level_button_group->buttons();
  for (auto button : buttons) {
    level_button_group->removeButton(button);
  }

  // add level selection buttons for all the levels in the new map
  for (size_t i = 0; i < map.levels.size(); i++) {
    QPushButton *button = new QPushButton(
        QString::fromStdString(map.levels[i].name));
    button->setCheckable(true);
    level_button_group->addButton(button, i);
    level_button_hbox_layout->addWidget(button);
  }
}

void Editor::new_map()
{
  QFileDialog dialog(this, "New Project");
  dialog.setNameFilter("*.yaml");
  dialog.setDefaultSuffix(".yaml");
  dialog.setAcceptMode(QFileDialog::AcceptMode::AcceptSave);
  dialog.setConfirmOverwrite(true);

  if (dialog.exec() != QDialog::Accepted)
    return;

  QFileInfo file_info(dialog.selectedFiles().first());
  std::string fn = file_info.fileName().toStdString();

  project_filename = file_info.fileName();
  QString dir_path = file_info.dir().path();
  QDir::setCurrent(dir_path);

  map.clear();
  update_level_buttons();
  save();
}

void Editor::open()
{
  QFileDialog dialog(this, "Open Project");
  dialog.setNameFilter("*.yaml");

  if (dialog.exec() != QDialog::Accepted)
    return;
  
  load_project(dialog.selectedFiles().first());
}

void Editor::save()
{
  if (project_filename.isEmpty()) {
    QFileDialog dialog(this, "Save Project");
    dialog.setNameFilter("*.yaml");
    dialog.setDefaultSuffix(".yaml");
    dialog.setAcceptMode(QFileDialog::AcceptMode::AcceptSave);
    dialog.setConfirmOverwrite(true);

    if (dialog.exec() != QDialog::Accepted) {
      QMessageBox::critical(
          this,
          "Project not saved",
          "Filename not supplied. Project not saved!");
      return;
    }

    QFileInfo file_info(dialog.selectedFiles().first());
    std::string fn = file_info.fileName().toStdString();

    project_filename = file_info.fileName();
    QString dir_path = file_info.dir().path();
    QDir::setCurrent(dir_path);
  }
  const std::string filename_std_string = project_filename.toStdString();
  map.save_yaml(filename_std_string);
}

void Editor::about()
{
  QMessageBox::about(this, "about", "hello world");
}

void Editor::level_edit()
{
  LevelDialog level_dialog(this);
  level_dialog.exec();
}

void Editor::level_add()
{
  bool ok = false;
  QString level_name = QInputDialog::getText(
      this,
      "Add Level",
      "Level name:",
      QLineEdit::Normal,
      QString(),
      &ok);
  if (!ok || level_name.isEmpty())
    return;
  map.add_level(level_name.toStdString());
  update_level_buttons();
  level_button_group->button(map.levels.size()-1)->click();
}

void Editor::zoom_normal()
{
  //map_view->set_absolute_scale(1.0);
  map_view->resetMatrix();
}

void Editor::zoom_in()
{
  map_view->scale(1.25, 1.25);
}

void Editor::zoom_out()
{
  map_view->scale(0.8, 0.8);
}

void Editor::zoom_fit()
{
  zoom_normal();
}

void Editor::mouse_event(const MouseType t, QMouseEvent *e)
{
  QPointF p;
  if (!is_mouse_event_in_map(e, p)) {
    e->ignore();
    return;
  }
  if (level_idx >= static_cast<int>(map.levels.size()))
    return;
  // dispatch to individual mouse handler functions to save indenting...
  switch (tool_id) {
    case SELECT:       mouse_select(t, e, p); break;
    case ADD_VERTEX:   mouse_add_vertex(t, e, p); break;
    case MOVE_VERTEX:  mouse_move_vertex(t, e, p); break;
    case ADD_LANE:     mouse_add_lane(t, e, p); break;
    case ADD_WALL:     mouse_add_wall(t, e, p); break;
    case ADD_MEAS:     mouse_add_meas(t, e, p); break;
    case ADD_MODEL:    mouse_add_model(t, e, p); break;
    case ROTATE_MODEL: mouse_rotate_model(t, e, p); break;
    case MOVE_MODEL:   mouse_move_model(t, e, p); break;
    case ADD_FLOOR:    mouse_add_floor(t, e, p); break;
    case EDIT_POLYGON: mouse_edit_polygon(t, e, p); break;
    case ADD_ZONE:     mouse_add_zone(t, e, p); break;
    default: break;
  }
}

void Editor::mousePressEvent(QMouseEvent *e)
{
  mouse_event(PRESS, e);
}

void Editor::mouseReleaseEvent(QMouseEvent *e)
{
  mouse_event(RELEASE, e);
}

void Editor::mouseMoveEvent(QMouseEvent *e)
{
  mouse_event(MOVE, e);
}

int Editor::get_polygon_idx(const double x, const double y)
{
  const Level &level = map.levels[level_idx];
  for (size_t i = 0; i < level.polygons.size(); i++) {
    const auto &polygon = level.polygons[i];
    QVector<QPointF> polygon_vertices;
    for (const auto &vertex_idx: polygon.vertices) {
      const Vertex &v = level.vertices[vertex_idx];
      polygon_vertices.append(QPointF(v.x, v.y));
    }
    QPolygonF qpolygon(polygon_vertices);
    if (qpolygon.containsPoint(QPoint(x, y), Qt::OddEvenFill))
      return static_cast<int>(i);
  }
  return -1;  // not found
}

bool Editor::is_mouse_event_in_map(QMouseEvent *e, QPointF &p_scene)
{
  const QPoint p_global = mapToGlobal(e->pos());
  const QPoint p_map = map_view->mapFromGlobal(p_global);
  if (p_map.x() < 0 || p_map.y() < 0 ||
      p_map.x() >= map_view->width() || p_map.y() >= map_view->height())
    return false;
  // This event point is valid. Now we can set p_scene.
  p_scene = map_view->mapToScene(p_map);
  return true;
}

void Editor::keyPressEvent(QKeyEvent *e)
{
  switch (e->key()) {
    case Qt::Key_Delete:
      map.delete_keypress(level_idx);
      create_scene();
      break;
    case Qt::Key_S:
    case Qt::Key_Escape:
      tool_button_group->button(SELECT)->click();
      clear_selection();
      update_property_editor();
      create_scene();
      break;
    case Qt::Key_V:
      tool_button_group->button(ADD_VERTEX)->click();
      break;
    case Qt::Key_M:
      tool_button_group->button(MOVE_VERTEX)->click();
      break;
    case Qt::Key_L:
      tool_button_group->button(ADD_LANE)->click();
      break;
    case Qt::Key_W:
      tool_button_group->button(ADD_WALL)->click();
      break;
    case Qt::Key_T:
      tool_button_group->button(ADD_MEAS)->click();
      break;
    case Qt::Key_O:
      tool_button_group->button(ADD_MODEL)->click();
      model_name_line_edit->setFocus(Qt::OtherFocusReason);
      break;
    case Qt::Key_R:
      tool_button_group->button(ROTATE_MODEL)->click();
      break;
    case Qt::Key_F:
      tool_button_group->button(ADD_FLOOR)->click();
      break;
    case Qt::Key_E:
      tool_button_group->button(EDIT_POLYGON)->click();
      break;
    case Qt::Key_D:
      tool_button_group->button(MOVE_MODEL)->click();
      break;
    case Qt::Key_Z:
      tool_button_group->button(ADD_ZONE)->click();
      break;
    case Qt::Key_B:
      for (auto &edge : map.levels[level_idx].edges) {
        if (edge.type == Edge::LANE && edge.selected) {
          // toggle bidirectional flag
          edge.set_param("bidirectional",
              edge.is_bidirectional() ? "false" : "true");
          create_scene();
        }
      }
      break;
    case Qt::Key_0: number_key_pressed(0); break;
    case Qt::Key_1: number_key_pressed(1); break;
    case Qt::Key_2: number_key_pressed(2); break;
    case Qt::Key_3: number_key_pressed(3); break;
    case Qt::Key_4: number_key_pressed(4); break;
    case Qt::Key_5: number_key_pressed(5); break;
    case Qt::Key_6: number_key_pressed(6); break;
    case Qt::Key_7: number_key_pressed(7); break;
    case Qt::Key_8: number_key_pressed(8); break;
    case Qt::Key_9: number_key_pressed(9); break;
    default:
      break;
  }
}

const QString Editor::tool_id_to_string(const int id)
{
  switch (id) {
    case SELECT: return "&select";
    case ADD_VERTEX: return "add &vertex";
    case MOVE_VERTEX: return "&move vertex";
    case ADD_LANE: return "add &lane";
    case ADD_WALL: return "add &wall";
    case ADD_MEAS: return "add measuremen&t";
    case ADD_MODEL: return "add m&odel";
    case MOVE_MODEL: return "move mo&del";
    case ROTATE_MODEL: return "&rotate model";
    case ADD_FLOOR: return "add &floor";
    case EDIT_POLYGON: return "&edit polygon";
    case ADD_ZONE: return "add &zone";
    default: return "unknown tool ID";
  }
}

void Editor::tool_toggled(int id, bool checked)
{
  if (!checked)
    return;

  clicked_idx = -1;
  remove_mouse_motion_item();

  tool_id = id;

  // set the cursor
  Qt::CursorShape cursor = Qt::ArrowCursor;
  if (tool_id == ADD_VERTEX || tool_id == ADD_MODEL)
    cursor = Qt::CrossCursor;
  map_view->setCursor(cursor);

  // set the status bar
  switch (id) {
    case SELECT:
      statusBar()->showMessage("Click an item to select it.");
      break;
    case ADD_LANE:
    case ADD_WALL:
    case ADD_MEAS:
      statusBar()->showMessage(
          "Click and drag from a vertex to another vertex to add an edge.");
      break;
    case ADD_FLOOR:
    case ADD_ZONE:
      statusBar()->showMessage(
          "Left-click to add polygon vertices. "
          "Right-click to close polygon.");
      break;
    case EDIT_POLYGON:
      statusBar()->showMessage(
          "Left-click drag an edge to introduce a new vertex. "
          "Right-click a polygon vertex to remove it from the polygon.");
      break;
    default:
      statusBar()->clearMessage();
      break;
  }
}

void Editor::update_property_editor()
{
  if (map.levels.empty())
    return;

  for (const auto &p : map.levels[level_idx].polygons) {
    if (p.selected) {
      printf("found a selected polygon\n");
      // todo: populate property editor
      return;
    }
  }
  for (const auto &e : map.levels[level_idx].edges) {
    if (e.selected) {
      printf("found a selected edge\n");
      populate_property_editor(e);
      return;  // stop after finding the first one
    }
  }
  for (const auto &m : map.levels[level_idx].models) {
    if (m.selected) {
      printf("found a selected model\n");
      populate_property_editor(m);
      return;  // stop after finding the first one
    }
  }
  for (const auto &v : map.levels[level_idx].vertices) {
    if (v.selected) {
      printf("found a selected vertex\n");
      populate_property_editor(v);
      return;  // stop after finding the first one
    }
  }
  // if we get here, we never found anything :(
  clear_property_editor();
}

QTableWidgetItem *Editor::create_table_item(
    const QString &str, bool editable)
{
  QTableWidgetItem *item = new QTableWidgetItem(str);
  if (!editable)
    item->setFlags(Qt::NoItemFlags);
  else
    item->setBackground(QBrush(Qt::white));
  return item;
}

void Editor::populate_property_editor(const Edge &edge)
{
  property_editor->blockSignals(true);  // otherwise we get tons of callbacks

  property_editor->setRowCount(3 + edge.params.size());

  property_editor->setItem(0, 0, create_table_item("edge_type"));
  property_editor->setItem(0, 1, create_table_item(
      QString::fromStdString(edge.type_to_string())));

  property_editor->setItem(1, 0, create_table_item("start_idx"));
  property_editor->setItem(1, 1, create_table_item(
      QString::number(edge.start_idx)));

  property_editor->setItem(2, 0, create_table_item("end_idx"));
  property_editor->setItem(2, 1, create_table_item(
      QString::number(edge.end_idx)));

  int row = 3;
  for (const auto &param : edge.params) {
    property_editor->setItem(row, 0, create_table_item(
        QString::fromStdString(param.first)));
    property_editor->setItem(row, 1, create_table_item(
        param.second.to_qstring(), true));
    row++;
  }

  property_editor->blockSignals(false);  // re-enable callbacks
}

void Editor::populate_property_editor(const Vertex &vertex)
{
  property_editor->blockSignals(true);  // otherwise we get tons of callbacks
  property_editor->setRowCount(3);

  property_editor->setItem(0, 0, create_table_item("x"));
  property_editor->setItem(0, 1, create_table_item(
      QString::number(vertex.x)));

  property_editor->setItem(1, 0, create_table_item("y"));
  property_editor->setItem(1, 1, create_table_item(
      QString::number(vertex.y)));

  property_editor->setItem(2, 0, create_table_item("name"));
  property_editor->setItem(2, 1, create_table_item(
      QString::fromStdString(vertex.name), true));

  property_editor->blockSignals(false);  // re-enable callbacks
}

void Editor::populate_property_editor(const Model &model)
{
  printf("populate_property_editor(model)\n");
  property_editor->blockSignals(true);  // otherwise we get tons of callbacks

  property_editor->setRowCount(2);
  property_editor->setItem(0, 0, create_table_item("name"));
  property_editor->setItem(0, 1, create_table_item(
      QString::fromStdString(model.instance_name)));

  property_editor->setItem(1, 0, create_table_item("model_name"));
  property_editor->setItem(1, 1, create_table_item(
      QString::fromStdString(model.model_name)));

  property_editor->blockSignals(false);  // re-enable callbacks
}

void Editor::clear_property_editor()
{
  property_editor->setRowCount(0);
}

void Editor::property_editor_cell_changed(int row, int column)
{
  std::string name = property_editor->item(row, 0)->text().toStdString();
  std::string value = property_editor->item(row, 1)->text().toStdString();
  printf("property_editor_cell_changed(%d, %d) = param %s\n",
      row, column, name.c_str());

  for (auto &v : map.levels[level_idx].vertices) {
    if (!v.selected)
      continue;
    if (name == "name") {
      v.name = value;
    }
    create_scene();
    return;  // stop after finding the first one
  }

  for (auto &e : map.levels[level_idx].edges) {
    if (!e.selected)
      continue;
    e.set_param(name, value);
    create_scene();
    return;  // stop after finding the first one
  }
}

void Editor::model_name_line_edited(const QString &text)
{
  //qDebug("model_name_line_edited(%s)", qUtf8Printable(text));
  if (model_name_list_widget->count() == 0) {
    qWarning("model name list widget is empty :(");
    return;  // nothing to do; there is no available model list
  }

  // see if we can auto-complete with anything in the list box
  // scroll the list box to the first thing
  std::string user_text_lower(text.toLower().toStdString());
  // could become super fancy but for now let's just do linear search...
  size_t closest_idx = 0;
  for (size_t i = 0; i < models.size(); i++) {
    if (user_text_lower < models[i].name_lowercase) {
      closest_idx = i;
      break;
    }
  }
  QListWidgetItem *item = model_name_list_widget->item(closest_idx);
  model_name_list_widget->setCurrentItem(item);
  model_name_list_widget->scrollToItem(
      item,
      QAbstractItemView::PositionAtTop);
}

void Editor::model_name_list_widget_changed(int row)
{
  qDebug("model_name_list_widget_changed(%d)", row);
  const QPixmap &model_pixmap = models[row].get_pixmap();
  if (model_pixmap.isNull())
    return;  // we don't have a pixmap to draw :(
  // scale the pixmap so it fits within the currently allotted space
  const int w = model_preview_label->width();
  const int h = model_preview_label->height();
  model_preview_label->setPixmap(
      model_pixmap.scaled(w, h, Qt::KeepAspectRatio));
}

bool Editor::create_scene()
{
  scene->clear();  // destroys the mouse_motion_* items if they are there
  mouse_motion_line = nullptr;
  mouse_motion_model = nullptr;
  mouse_motion_ellipse = nullptr;
  mouse_motion_polygon = nullptr;

  if (map.levels.empty()) {
    printf("nothing to draw!\n");
    return false;
  }

  const Level *level = &map.levels[level_idx];
  scene->setSceneRect(
      QRectF(0, 0, level->drawing_width, level->drawing_height));
  printf("setSceneRect(0, 0, %d, %d)\n", level->drawing_width, level->drawing_height);

  if (level->drawing_filename.size()) {
    scene->addPixmap(level->pixmap);
  }
  else {
    scene->addRect(
        0, 0,
        level->drawing_width, level->drawing_height,
        QPen(),
        Qt::white);
  }

  for (const auto &polygon : level->polygons) {
    // now draw the polygons
    QVector<QPointF> polygon_vertices;
    for (const auto &vertex_idx: polygon.vertices) {
      const Vertex &v = level->vertices[vertex_idx];
      polygon_vertices.append(QPointF(v.x, v.y));
    }
    scene->addPolygon(
        QPolygonF(polygon_vertices),
        QPen(Qt::black),
        polygon.selected ? selected_polygon_brush : polygon_brush);
  }

  level->draw_edges(scene);

  // now draw all the models
  for (const auto &nav_model : level->models) {
    // find the pixmap we need for this model
    QPixmap pixmap;
    double model_meters_per_pixel = 1.0;  // will get overridden
    for (auto &model : models) {
      if (model.name == nav_model.model_name) {
        pixmap = model.get_pixmap();
        model_meters_per_pixel = model.meters_per_pixel;
        break;
      }
    }
    if (pixmap.isNull())
      continue;  // couldn't load the pixmap; ignore it.

    QGraphicsPixmapItem *item = scene->addPixmap(pixmap);
    item->setOffset(-pixmap.width()/2, -pixmap.height()/2);
    item->setScale(model_meters_per_pixel / level->drawing_meters_per_pixel);
    item->setPos(nav_model.x, nav_model.y);
    item->setRotation(-nav_model.yaw * 180.0 / M_PI);
  }

  level->draw_vertices(scene);

#if 0
  // ahhhhh only for debugging...
  // plot the nearest projection point to a polygon, if it's set
  // to something nonzero
  if (level->polygon_edge_proj_x != 0) {
    const double r = 5.0;
    addEllipse(
        level->polygon_edge_proj_x - r,
        level->polygon_edge_proj_y - r,
        2 * r,
        2 * r,
        QPen(Qt::black),
        QBrush(Qt::blue));
  }
#endif

  return true;
}

void Editor::clear_selection()
{
  if (map.levels.empty())
    return;
  for (auto &vertex: map.levels[level_idx].vertices)
    vertex.selected = false;
  for (auto &edge: map.levels[level_idx].edges)
    edge.selected = false;
  for (auto &model: map.levels[level_idx].models)
    model.selected = false;
  for (auto &polygon: map.levels[level_idx].polygons)
    polygon.selected = false;
}

void Editor::draw_mouse_motion_line_item(
    const double mouse_x,
    const double mouse_y)
{
  double pen_width = 1;
  QColor color;
  switch (tool_id) {
    case ADD_LANE:
      pen_width = 20;
      color = QColor::fromRgbF(0, 0, 1, 0.5); 
      break;
    case ADD_WALL:
      pen_width = 5;
      color = QColor::fromRgbF(0, 0, 1, 0.5); 
      break;
    case ADD_MEAS:
      pen_width = 5;
      color = QColor::fromRgbF(1, 0, 1, 0.5); 
      break;
  }

  QPen pen(QBrush(color), pen_width, Qt::SolidLine, Qt::RoundCap);
  const auto &start = map.levels[level_idx].vertices[clicked_idx];
  if (!mouse_motion_line) {
    mouse_motion_line = scene->addLine(start.x, start.y, mouse_x, mouse_y, pen);
  }
  else {
    mouse_motion_line->setLine(start.x, start.y, mouse_x, mouse_y);
  }
}

void Editor::remove_mouse_motion_item()
{
  if (mouse_motion_line) {
    scene->removeItem(mouse_motion_line);
    delete mouse_motion_line;
    mouse_motion_line = nullptr;
  }
  if (mouse_motion_model) {
    scene->removeItem(mouse_motion_model);
    delete mouse_motion_model;
    mouse_motion_model = nullptr;
  }
  if (mouse_motion_ellipse) {
    scene->removeItem(mouse_motion_ellipse);
    delete mouse_motion_ellipse;
    mouse_motion_ellipse = nullptr;
  }
  if (mouse_motion_polygon) {
    scene->removeItem(mouse_motion_polygon);
    delete mouse_motion_polygon;
    mouse_motion_polygon = nullptr;
  }
}

void Editor::set_selected_line_item(QGraphicsLineItem *line_item)
{
  clear_selection();

  if (line_item == nullptr)
    return;

  // find if any of our lanes match those vertices
  for (auto &edge : map.levels[level_idx].edges) {
    const auto &v_start = map.levels[level_idx].vertices[edge.start_idx];
    const auto &v_end = map.levels[level_idx].vertices[edge.end_idx];
    if (line_vertices_match(line_item, v_start, v_end, 10.0)) {
      edge.selected = true;
      return;  // stop after first one is found, don't select multiple
    }
  }
}

void Editor::set_selected_pixmap_item(QGraphicsPixmapItem *item)
{
  clear_selection();
  if (item == nullptr)
    return;
  // find which model matches the vertex
  for (auto &model : map.levels[level_idx].models) {
    const double dx = item->pos().x() - model.x;
    const double dy = item->pos().y() - model.y;
    const double dist = sqrt(dx*dx + dy*dy);
    // should be exactly the same vertex, but give some room for
    // floating-point roundoff and stuff.
    if (dist < 1.0) {
      model.selected = true;
      return;
    }
  }
}

void Editor::set_selected_ellipse_item(QGraphicsEllipseItem *item)
{
  clear_selection();
  if (item == nullptr)
    return;
  for (auto &vertex : map.levels[level_idx].vertices) {
    const double dx = item->rect().center().x() - vertex.x;
    const double dy = item->rect().center().y() - vertex.y;
    const double dist = sqrt(dx*dx + dy*dy);
    // should be exactly the same vertex, but give some room for
    // floating-point roundoff and stuff.
    if (dist < 1.0) {
      vertex.selected = true;
      return;
    }
  }
}

bool Editor::line_vertices_match(
    const QGraphicsLineItem *line_item,
    const Vertex &v1,
    const Vertex &v2,
    const double max_dist)
{
  if (!line_item)
    return false;

  // look up the line's vertices
  const double x1 = line_item->line().x1();
  const double y1 = line_item->line().y1();
  const double x2 = line_item->line().x2();
  const double y2 = line_item->line().y2();

  // calculate distances
  const double dx1 = v1.x - x1;
  const double dy1 = v1.y - y1;
  const double dx2 = v2.x - x2;
  const double dy2 = v2.y - y2;
  const double v1_dist = sqrt(dx1*dx1 + dy1*dy1);
  const double v2_dist = sqrt(dx2*dx2 + dy2*dy2);

  return v1_dist < max_dist && v2_dist < max_dist;
}

///////////////////////////////////////////////////////////////////////
// MOUSE HANDLERS
///////////////////////////////////////////////////////////////////////

void Editor::mouse_select(
    const MouseType type, QMouseEvent *e, const QPointF &p)
{
  if (type != PRESS)
    return;
  clear_selection();
  const QPoint p_global = mapToGlobal(e->pos());
  const QPoint p_map = map_view->mapFromGlobal(p_global);
  QGraphicsItem *item = map_view->itemAt(p_map.x(), p_map.y());
  if (item) {
    if (item->type() == QGraphicsLineItem::Type) {
      set_selected_line_item(
          qgraphicsitem_cast<QGraphicsLineItem *>(item));
    }
    else if (item->type() == QGraphicsEllipseItem::Type) {
      set_selected_ellipse_item(
          qgraphicsitem_cast<QGraphicsEllipseItem *>(item));
    }
    else if (item->type() == QGraphicsPixmapItem::Type) {
      set_selected_pixmap_item(
          qgraphicsitem_cast<QGraphicsPixmapItem *>(item));
    }
    else if (item->type() == QGraphicsPolygonItem::Type) {
      polygon_idx = get_polygon_idx(p.x(), p.y());
      if (polygon_idx < 0)
        return;  // didn't click on a polygon
      Polygon &polygon = map.levels[level_idx].polygons[polygon_idx];
      polygon.selected = true;
      for (const auto &vertex_idx : polygon.vertices)
        map.levels[level_idx].vertices[vertex_idx].selected = true;
    }
  }
  // todo: be smarter and go find the actual GraphicsItem to avoid
  // a full repaint here...
  create_scene();
  update_property_editor();
}

void Editor::mouse_add_vertex(
    const MouseType t, QMouseEvent *, const QPointF &p)
{
  if (t == PRESS) {
    map.add_vertex(level_idx, p.x(), p.y());
    create_scene();
  }
}

void Editor::mouse_move_vertex(
    const MouseType t, QMouseEvent *, const QPointF &p)
{
  if (t == PRESS) {
    clicked_idx = map.nearest_item_index_if_within_distance(
        level_idx, p.x(), p.y(), 10.0, Map::VERTEX);
  }
  else if (t == RELEASE) {
    clicked_idx = -1;
  }
  else if (t == MOVE) {
    if (clicked_idx < 0)
      return;
    Vertex *pt = &map.levels[level_idx].vertices[clicked_idx];
    pt->x = p.x();
    pt->y = p.y();
    create_scene();
  }
}

void Editor::mouse_add_edge(
    const MouseType t,
    QMouseEvent *,
    const QPointF &p,
    const Edge::Type &edge_type)
{
  if (t == PRESS) {
    clicked_idx = map.nearest_item_index_if_within_distance(
        level_idx, p.x(), p.y(), 10.0, Map::VERTEX);
  }
  else if (t == RELEASE) {
    if (clicked_idx < 0)
      return;
    remove_mouse_motion_item();
    double distance = 0;
    const int release_idx = map.find_nearest_vertex_index(
        level_idx, p.x(), p.y(), distance);
    if (distance > 10.0 || (clicked_idx == release_idx)) {
      clicked_idx = -1;
      return;
    }
    map.add_edge(level_idx, clicked_idx, release_idx, edge_type);
    clicked_idx = -1;
    create_scene();
  }
  else if (t == MOVE) {
    if (clicked_idx < 0)
      return;
    draw_mouse_motion_line_item(p.x(), p.y());
  }
}

void Editor::mouse_add_lane(
    const MouseType t, QMouseEvent *e, const QPointF &p)
{
  mouse_add_edge(t, e, p, Edge::LANE);
}

void Editor::mouse_add_wall(
    const MouseType t, QMouseEvent *e, const QPointF &p)
{
  mouse_add_edge(t, e, p, Edge::WALL);
}

void Editor::mouse_add_meas(
    const MouseType t, QMouseEvent *e, const QPointF &p)
{
  mouse_add_edge(t, e, p, Edge::MEAS);
}

void Editor::mouse_add_model(
    const MouseType t, QMouseEvent *, const QPointF &p)
{
  if (t == PRESS) {
    const int model_row = model_name_list_widget->currentRow();
    if (model_row < 0)
      return;  // nothing currently selected. nothing to do.
    map.add_model(level_idx, p.x(), p.y(), 0.0, models[model_row].name);
    create_scene();
  }
  else if (t == MOVE) {
    const int model_row = model_name_list_widget->currentRow();
    if (model_row < 0)
      return;  // nothing currently selected. nothing to do.
    EditorModel &model = models[model_row];
    if (mouse_motion_model == nullptr) {
      const QPixmap pixmap(model.get_pixmap());
      mouse_motion_model = scene->addPixmap(pixmap);
      mouse_motion_model->setOffset(-pixmap.width()/2, -pixmap.height()/2);
      mouse_motion_model->setScale(
          model.meters_per_pixel /
          map.levels[level_idx].drawing_meters_per_pixel);
    }
    mouse_motion_model->setPos(p.x(), p.y());
  }
}

void Editor::mouse_rotate_model(
    const MouseType t, QMouseEvent *, const QPointF &p)
{
  if (t == PRESS) {
    clicked_idx = map.nearest_item_index_if_within_distance(
        level_idx,
        p.x(),
        p.y(),
        50.0,
        Map::MODEL);
    if (clicked_idx < 0)
      return; // nothing to do. click wasn't on a model.

    // find the model in the scene graph and remove it, and replace with the
    // mouse_motion_model item and mouse_motion_line
    const Model &model = map.levels[level_idx].models[clicked_idx];
    // TODO find previous QPixmapItem for this model and rotate it?
    // For now, we'll just rotate the heading indicator line.
    // Not as awesome, but it works.
  
    QPen pen(Qt::red);
    pen.setWidth(4);
    const double r = static_cast<double>(ROTATION_INDICATOR_RADIUS);
    mouse_motion_ellipse = scene->addEllipse(
        model.x - r,  // ellipse upper-left column
        model.y - r,  // ellipse upper-left row
        2 * r,  // ellipse width
        2 * r,  // ellipse height
        pen);
    mouse_motion_line = scene->addLine(
        model.x,
        model.y,
        model.x + r * cos(model.yaw),
        model.y + r * sin(model.yaw),
        pen);
  }
  else if (t == RELEASE) {
    remove_mouse_motion_item();
    if (clicked_idx < 0)
      return;
    map.rotate_model(level_idx, clicked_idx, p.x(), p.y());
    clicked_idx = -1;  // we're done rotating it now
    // now re-render the whole map (could optimize in the future...)
    create_scene();
  }
  else if (t == MOVE) {
    if (clicked_idx < 0)
      return;  // nothing currently selected. nothing to do.

    // re-orient the mouse_motion_model item and heading indicator as needed
    const Model &model = map.levels[level_idx].models[clicked_idx];
    const double dx = p.x() - model.x;
    const double dy = -(p.y() - model.y);  // vertical axis is flipped
    const double mouse_yaw = atan2(dy, dx);
    const double r = static_cast<double>(ROTATION_INDICATOR_RADIUS);
    mouse_motion_line->setLine(
        model.x,
        model.y,
        model.x + r * cos(mouse_yaw),
        model.y - r * sin(mouse_yaw));
  }
}

void Editor::mouse_move_model(
    const MouseType t, QMouseEvent *e, const QPointF &p)
{
  if (t == PRESS) {
    const double click_distance = 50.0;
    clicked_idx = map.nearest_item_index_if_within_distance(
        level_idx, p.x(), p.y(), click_distance, Map::MODEL);
    if (clicked_idx < 0)
      return;  // didn't click close to an existing model
    // Now we need to find the pixmap item for this model.
    // todo: use fancier calls if the scene graph gets so big that a linear
    // search becomes intolerably slow
    const QList <QGraphicsItem *> items = scene->items();
    mouse_motion_model = nullptr;
    double min_dist = 1.0e9;
    QPointF mp(
        map.levels[level_idx].models[clicked_idx].x,
        map.levels[level_idx].models[clicked_idx].y);
    for (const auto item : items) {
      if (item->type() != QGraphicsPixmapItem::Type)
        continue;  // ignore anything other than the pixmaps (models)
      const double model_click_distance = QLineF(mp, item->pos()).length();
      if (model_click_distance < min_dist) {
        min_dist = model_click_distance;
        mouse_motion_model = qgraphicsitem_cast<QGraphicsPixmapItem *>(item);
      }
    }
  }
  else if (t == RELEASE) {
    clicked_idx = -1;
    create_scene();
  }
  else if (t == MOVE) {
    if (!(e->buttons() & Qt::LeftButton))
      return;  // we only care about mouse-dragging, not just motion
    if (clicked_idx < 0 || mouse_motion_model == nullptr)
      return;  // nothing was clicked before the drag started
    // update both the nav_model data and the pixmap in the scene
    map.levels[level_idx].models[clicked_idx].x = p.x();
    map.levels[level_idx].models[clicked_idx].y = p.y();
    mouse_motion_model->setPos(p);
  }
}

void Editor::mouse_add_polygon(
    const MouseType t,
    QMouseEvent *e,
    const QPointF &p,
    const Polygon::Type &polygon_type)
{
  if (t == PRESS) {
    if (e->buttons() & Qt::LeftButton) {
      clicked_idx = map.nearest_item_index_if_within_distance(
          level_idx, p.x(), p.y(), 10.0, Map::VERTEX);
      if (clicked_idx < 0)
        return; // nothing to do. click wasn't on a vertex.

      Vertex &v = map.levels[level_idx].vertices[clicked_idx];
      v.selected = true;  // todo: find graphics item for vertex and colorize it
    
      if (mouse_motion_polygon == nullptr) {
        QVector<QPointF> polygon_vertices;
        polygon_vertices.append(QPointF(v.x, v.y));
        QPolygonF polygon(polygon_vertices);
        mouse_motion_polygon = scene->addPolygon(
            polygon, QPen(Qt::black), selected_polygon_brush);
        mouse_motion_polygon_vertices.clear();
      }
    
      // only add vertex_idx is NOT already in the vertex list
      if (std::find(
          mouse_motion_polygon_vertices.begin(),
          mouse_motion_polygon_vertices.end(),
          clicked_idx) == mouse_motion_polygon_vertices.end()) {
        mouse_motion_polygon_vertices.push_back(clicked_idx);
      }
    }
    else if (e->buttons() & Qt::RightButton) {
      if (mouse_motion_polygon == nullptr)
        return;
      if (mouse_motion_polygon_vertices.size() >= 3) {
        Polygon polygon;
        polygon.type = polygon_type;
        for (const auto &i : mouse_motion_polygon_vertices)
          polygon.vertices.push_back(i);
        map.levels[level_idx].polygons.push_back(polygon);
      }
      scene->removeItem(mouse_motion_polygon);
      delete mouse_motion_polygon;
      mouse_motion_polygon = nullptr;

      clear_selection();
      create_scene();
    }
  }
  else if (t == MOVE) {
    if (mouse_motion_polygon == nullptr)
      return;  // not sure how we got here

    // first, remove the previous polygon
    scene->removeItem(mouse_motion_polygon);
    delete mouse_motion_polygon;
 
    // now, make the updated polygon
    QVector<QPointF> polygon_vertices;
    for (const auto &vertex_idx: mouse_motion_polygon_vertices) {
      const Vertex &v = map.levels[level_idx].vertices[vertex_idx];
      polygon_vertices.append(QPointF(v.x, v.y));
    }
    polygon_vertices.append(QPointF(p.x(), p.y()));
 
    // insert the updated polygon into the scene
    QPolygonF polygon(polygon_vertices);
    mouse_motion_polygon = scene->addPolygon(
        polygon, QPen(Qt::black), selected_polygon_brush);
  }
}

void Editor::mouse_add_floor(
    const MouseType t, QMouseEvent *e, const QPointF &p)
{
  mouse_add_polygon(t, e, p, Polygon::FLOOR);
}

void Editor::mouse_add_zone(
    const MouseType t, QMouseEvent *e, const QPointF &p)
{
  mouse_add_polygon(t, e, p, Polygon::ZONE);
}

void Editor::mouse_edit_polygon(
    const MouseType t, QMouseEvent *e, const QPointF &p)
{
  if (t == PRESS) {
    if (e->buttons() & Qt::RightButton) {
      if (polygon_idx < 0)
        return;  // no polygon is selected, nothing to do
      int vertex_idx = map.nearest_item_index_if_within_distance(
          level_idx, p.x(), p.y(), 10.0, Map::VERTEX);
      if (vertex_idx < 0)
        return;  // Nothing to do. Click wasn't near a vertex.
      // first mark the vertex as no longer selected
      map.remove_polygon_vertex(level_idx, polygon_idx, vertex_idx);
      create_scene();
    }
    else if (e->buttons() & Qt::LeftButton) {
      // figure out which edge (if any) we are nearest on this polygon
      if (polygon_idx < 0)
        return;  // nothing to do; no polygon currently selected
      mouse_motion_polygon_vertex_idx = -1;
      const double x = p.x();
      const double y = p.y();
      const int polygon_vertex_drag_idx =
          map.polygon_edge_drag_press(level_idx, polygon_idx, x, y);
      if (polygon_vertex_drag_idx < 0)
        return;
    
      if (mouse_motion_polygon != nullptr) {
        qWarning("edit_polygon_release() without null mouse_motion_polygon!");
        return;
      }
    
      // create the mouse motion polygon and insert a new edge
      QVector<QPointF> polygon_vertices;
      const Polygon &polygon =
          map.levels[level_idx].polygons[polygon_idx];
      for (size_t i = 0; i < polygon.vertices.size(); i++) {
        const int v_idx = polygon.vertices[i];
        const Vertex &v = map.levels[level_idx].vertices[v_idx];
        polygon_vertices.append(QPointF(v.x, v.y));
        if (v_idx == polygon_vertex_drag_idx) {
          polygon_vertices.append(QPointF(x, y));  // current mouse location
          mouse_motion_polygon_vertex_idx = i + 1;
        }
      }
      QPolygonF drag_polygon(polygon_vertices);
    
      mouse_motion_polygon = scene->addPolygon(
          drag_polygon, QPen(Qt::black), polygon_brush);
    
      //create_scene();
    }
  }
  else if (t == RELEASE) {
    // todo by drag mode (left/right button?)
    if (mouse_motion_polygon == nullptr) {
      qInfo("woah! edit_polygon_release() with null mouse_motion_polygon!");
      return;
    }
    qInfo("replacing vertices of polygon %d...", polygon_idx);
    QPolygonF polygon = mouse_motion_polygon->polygon();
    scene->removeItem(mouse_motion_polygon);
    delete mouse_motion_polygon;
    mouse_motion_polygon = nullptr;
  
    int release_vertex_idx = map.nearest_item_index_if_within_distance(
        level_idx, p.x(), p.y(), 10.0, Map::VERTEX);
    if (release_vertex_idx < 0)
      return;  // nothing to do; didn't release near a vertex
  
    Polygon &existing = map.levels[level_idx].polygons[polygon_idx];
    if (std::find(
        existing.vertices.begin(),
        existing.vertices.end(),
        release_vertex_idx) != existing.vertices.end())
      return;  // Release vertex is already in the polygon. Don't do anything.
  
    existing.vertices.insert(
        existing.vertices.begin() + mouse_motion_polygon_vertex_idx,
        release_vertex_idx);
  
    create_scene();
  }
  else if (t == MOVE) {
    if (e->buttons() & Qt::LeftButton) {
      if (mouse_motion_polygon == nullptr) {
        qInfo("woah! edit_polygon_release() with null mouse_motion_polygon!");
        return;
      }
      QPolygonF polygon = mouse_motion_polygon->polygon();
      polygon[mouse_motion_polygon_vertex_idx] = QPointF(p.x(), p.y());
      mouse_motion_polygon->setPolygon(polygon);
    }
  }
}

void Editor::level_button_toggled(int button_idx, bool checked)
{
  qInfo("level button toggled: %d, %d", button_idx, checked ? 1 : 0);
  if (!checked)
    return;
  level_idx = button_idx;
  create_scene();
}

void Editor::number_key_pressed(const int n)
{
  for (auto &edge : map.levels[level_idx].edges) {
    if (edge.selected && edge.type == Edge::LANE)
      edge.set_graph_idx(n);
  }
  create_scene();
  update_property_editor();
}
