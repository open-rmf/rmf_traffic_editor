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

#include <algorithm>
#include <cmath>
#include <string>

#include <QtWidgets>

#include <QInputDialog>
#include <QLabel>
#include <QListWidget>
#include <QToolBar>

#include <yaml-cpp/yaml.h>

#include "add_param_dialog.h"
#include "building_dialog.h"
#include "building_level_dialog.h"
#include "editor.h"
#include "layer_dialog.h"
#include "map_view.h"
#include "model_dialog.h"
#include "preferences_dialog.h"
#include "preferences_keys.h"
using std::string;


Editor *Editor::instance = nullptr;

Editor::Editor()
: QMainWindow()
{
  instance = this;
  
  setWindowTitle("Traffic Editor[*]");

  QSettings settings;
  qDebug("settings filename: [%s]", qUtf8Printable(settings.fileName()));

  scene = new QGraphicsScene(this);

  map_view = new MapView(this);
  map_view->setScene(scene);

  QVBoxLayout *map_layout = new QVBoxLayout;
  map_layout->addWidget(map_view);

  layers_table = new TableList;  // todo: replace with specific subclass?

  level_table = new BuildingLevelTable;
  connect(
      level_table, &QTableWidget::cellClicked,
      [=](int row, int /*col*/) {
        if (row < static_cast<int>(project.building.levels.size()))
        {
          // save the center point of the current level's image coordinates
          const QPoint p_center_window(
              map_view->viewport()->width() / 2,
              map_view->viewport()->height() / 2);
          const QPointF p_center_scene = map_view->mapToScene(p_center_window);
          // printf("p_center_scene: (%.1f, %.1f)\n",
          //   p_center_scene.x(), p_center_scene.y());

          QPointF p_transformed;
          project.building.transform_between_levels(
              level_idx,
              p_center_scene,
              row,
              p_transformed);
          
          // maintain the view scale
          const double prev_scale = map_view->transform().m11();

          const double scale = prev_scale *
              project.building.levels[row].drawing_meters_per_pixel /
              project.building.levels[level_idx].drawing_meters_per_pixel;

          level_idx = row;
          create_scene();

          QTransform t;
          t.scale(scale, scale);
          map_view->setTransform(t);
          map_view->centerOn(p_transformed);
        }
      });

  connect(
      level_table,
      &BuildingLevelTable::redraw_scene,
      this,
      &Editor::create_scene);

  lift_table = new LiftTable;
  connect(
      lift_table,
      &TableList::redraw,
      [this]()
      {
        this->create_scene();
      });

  right_tab_widget = new QTabWidget;
  right_tab_widget->setStyleSheet("QTabBar::tab { color: white; }");
  right_tab_widget->addTab(level_table, "levels");
  right_tab_widget->addTab(layers_table, "layers");
  right_tab_widget->addTab(lift_table, "lifts");

  property_editor = new QTableWidget;
  property_editor->setStyleSheet("QTableWidget { background-color: #e0e0e0; color: black; gridline-color: #606060; } QLineEdit { background:white; }");
  property_editor->setMinimumSize(500, 200);
  property_editor->setSizePolicy(
      QSizePolicy::Fixed,
      QSizePolicy::MinimumExpanding);
  property_editor->setColumnCount(2);
  property_editor->horizontalHeader()->setVisible(false);
  property_editor->verticalHeader()->setVisible(false);
  property_editor->horizontalHeader()->setSectionResizeMode(
      0,
      QHeaderView::ResizeToContents);
  property_editor->horizontalHeader()->setSectionResizeMode(
      1,
      QHeaderView::Stretch);
  property_editor->verticalHeader()->setSectionResizeMode(
      QHeaderView::ResizeToContents);
  property_editor->setAutoFillBackground(true);
  connect(
      property_editor, &QTableWidget::cellChanged,
      this, &Editor::property_editor_cell_changed);

  QHBoxLayout *param_button_layout = new QHBoxLayout;

  add_param_button = new QPushButton("Add...");
  add_param_button->setEnabled(false);
  connect(
      add_param_button, &QAbstractButton::clicked,
      this, &Editor::add_param_button_clicked);

  delete_param_button = new QPushButton("Delete");
  delete_param_button->setEnabled(false);
  connect(
      delete_param_button, &QAbstractButton::clicked,
      this, &Editor::delete_param_button_clicked);

  param_button_layout->addWidget(add_param_button);
  param_button_layout->addWidget(delete_param_button);

  QVBoxLayout *right_column_layout = new QVBoxLayout;
  right_column_layout->addWidget(right_tab_widget);

  QLabel *properties_label = new QLabel("Properties");
  properties_label->setStyleSheet("QLabel { color: white; }");
  right_column_layout->addWidget(properties_label);
  right_column_layout->addWidget(property_editor);
  right_column_layout->addLayout(param_button_layout);

  QHBoxLayout *hbox_layout = new QHBoxLayout;
  hbox_layout->addLayout(map_layout, 1);
  hbox_layout->addLayout(right_column_layout);

  QWidget *w = new QWidget();
  w->setMouseTracking(true);
  setMouseTracking(true);
  w->setLayout(hbox_layout);
  w->setStyleSheet("background-color: #404040" );
  setCentralWidget(w);

  // FILE MENU
  QMenu *file_menu = menuBar()->addMenu("&File");

  file_menu->addAction(
      "&New Project...",
      this,
      &Editor::file_new_project,
      QKeySequence(Qt::CTRL + Qt::Key_N));

  file_menu->addAction(
      "&Open Project...",
      this,
      &Editor::file_open_project,
      QKeySequence(Qt::CTRL + Qt::Key_O));

  file_menu->addAction(
      "&Save Project",
      this,
      &Editor::file_save_project,
      QKeySequence(Qt::CTRL + Qt::Key_S));

  file_menu->addSeparator();

  file_menu->addAction(
      "E&xit",
      this,
      &QWidget::close,
      QKeySequence(Qt::CTRL + Qt::Key_Q));

  // EDIT MENU
  QMenu *edit_menu = menuBar()->addMenu("&Edit");
  edit_menu->addAction(
      "&Building properties...",
      this,
      &Editor::edit_building_properties);
  edit_menu->addSeparator();
  edit_menu->addAction("&Preferences...", this, &Editor::edit_preferences);

  // MODE MENU
  QMenu *mode_menu = menuBar()->addMenu("&Mode");
  mode_menu->addAction(
      "&Building",
      this,
      &Editor::mode_building,
      QKeySequence(Qt::CTRL + Qt::Key_B));

  mode_menu->addAction(
      "&Traffic lanes",
      this,
      &Editor::mode_traffic,
      QKeySequence(Qt::CTRL + Qt::Key_T));

  // VIEW MENU
  QMenu *view_menu = menuBar()->addMenu("&View");

  zoom_fit_action =
      view_menu->addAction("&Fit to Window", this, &Editor::zoom_fit);
  zoom_fit_action->setEnabled(false);

  // HELP MENU
  QMenu *help_menu = menuBar()->addMenu("&Help");

  help_menu->addAction("&About", this, &Editor::help_about);
  help_menu->addAction("About &Qt", &QApplication::aboutQt);

  // TOOLBAR
  toolbar = new QToolBar();

  mode_combo_box = new QComboBox;
  mode_combo_box->addItem("Building");
  mode_combo_box->addItem("Traffic");
  connect(
      mode_combo_box,
      &QComboBox::currentTextChanged,
      [this](const QString& text)
      {
        if (text == "Building")
          mode_building();
        else if (text == "Traffic")
          mode_traffic();
      });

  QLabel *mode_label = new QLabel("Edit mode:");
  mode_label->setStyleSheet("QLabel { color: white; }");
  toolbar->addWidget(mode_label);
  toolbar->addWidget(mode_combo_box);

  tool_button_group = new QButtonGroup(this);
  tool_button_group->setExclusive(true);

  create_tool_button(TOOL_SELECT, ":icons/select.svg", "Select (Esc)");
  create_tool_button(TOOL_MOVE, ":icons/move.svg", "Move (M)");
  create_tool_button(TOOL_ROTATE, ":icons/rotate.svg", "Rotate (R)");

  create_tool_button(
      TOOL_ADD_VERTEX,
      ":icons/add_vertex.svg",
      "Add Vertex (V)");
  create_tool_button(
      TOOL_ADD_FIDUCIAL,
      ":icons/fiducial.svg",
      "Add Fiducial");
  create_tool_button(TOOL_ADD_LANE, "", "Add Lane (L)");
  create_tool_button(TOOL_ADD_WALL, "", "Add Wall (W)");
  create_tool_button(TOOL_ADD_MEAS, "", "Add Measurement");
  create_tool_button(TOOL_ADD_DOOR, "", "Add Door");
  create_tool_button(TOOL_ADD_MODEL, "", "Add Model");
  create_tool_button(TOOL_ADD_FLOOR, "", "Add Floor Polygon");
  create_tool_button(TOOL_EDIT_POLYGON, "", "Edit Polygon");

  connect(
      tool_button_group,
      QOverload<int, bool>::of(&QButtonGroup::buttonToggled),
      this, &Editor::tool_toggled);

  toolbar->setStyleSheet("QToolBar {background-color: #404040; border: none; spacing: 5px} QToolButton {background-color: #c0c0c0; color: blue; border: 1px solid black;} QToolButton:checked {background-color: #808080; color: red; border: 1px solid black;}");
  addToolBar(Qt::TopToolBarArea, toolbar);

  ///////////////////////////////////////////////////////////
  // SET SIZE
  const int width =
      settings.contains(preferences_keys::width) ?
      settings.value(preferences_keys::width).toInt() :
      QGuiApplication::primaryScreen()->availableSize().width();

  const int height =
      settings.contains(preferences_keys::height) ?
      settings.value(preferences_keys::height).toInt() :
      QGuiApplication::primaryScreen()->availableSize().height();

  const int left =
      settings.contains(preferences_keys::left) ?
      settings.value(preferences_keys::left).toInt() :
      0;

  const int top =
      settings.contains(preferences_keys::top) ?
      settings.value(preferences_keys::top).toInt() :
      0;

  setGeometry(left, top, width, height);
  map_view->adjustSize();

  // default tool is the "select" tool
  tool_button_group->button(TOOL_SELECT)->click();

  load_model_names();
  level_table->setCurrentCell(level_idx, 0);
}

void Editor::load_model_names()
{
  // This function may throw exceptions. Caller should be ready for them!

  QSettings settings;
  QString thumbnail_path(
      settings.value(preferences_keys::thumbnail_path).toString());
  if (thumbnail_path.isEmpty())
  {
    // Currently not sure how to do this the "right" way. For now assume
    // everybody is building from source, I guess (?).
    // todo: figure out something better in the future for binary installs
    thumbnail_path =
        QDir::cleanPath(
            QDir(QApplication::applicationDirPath()).filePath("../thumbnails")
        );
    settings.setValue(preferences_keys::thumbnail_path, thumbnail_path);
  }

  QString model_list_path = QDir(thumbnail_path).filePath("model_list.yaml");

  YAML::Node y;
  std::string filename(model_list_path.toStdString());
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
  for (YAML::const_iterator it = ym.begin(); it != ym.end(); ++it)
    editor_models.push_back(
        EditorModel(it->as<std::string>(), model_meters_per_pixel));
}

QToolButton *Editor::create_tool_button(
    const int id,
    const QString& icon_filename,
    const QString & tooltip)
{
  QToolButton *b = new QToolButton(toolbar);
  b->setCheckable(true);

  if (!icon_filename.isEmpty())
  {
    QIcon icon(icon_filename);
    b->setIcon(icon);
    b->setToolTip(tooltip);
  }
  else
  {
    b->setText(tool_id_to_string(id));
    //b->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Maximum);
  }
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
  if (!project.load_yaml_file(filename.toStdString()))
    return false;

  level_idx = 0;

  if (!project.building.levels.empty())
  {
    const BuildingLevel& level = project.building.levels[level_idx];
    scene->setSceneRect(
        QRectF(0, 0, level.drawing_width, level.drawing_height));
    previous_mouse_point = QPointF(level.drawing_width, level.drawing_height);
  }

  create_scene();
  map_view->zoom_fit(project.building, level_idx);
  populate_layers_table();
  level_table->update(project.building);
  lift_table->update(project.building);

  QSettings settings;
  settings.setValue(preferences_keys::previous_project_path, filename);

  setWindowModified(false);

  return true;
}

bool Editor::load_previous_project()
{
  QSettings settings;
  const QString filename(
      settings.value(preferences_keys::previous_project_path).toString());
  if (!filename.isEmpty())
    return load_project(filename);
  return true;
}

void Editor::file_new_project()
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

  project.filename = file_info.absoluteFilePath().toStdString();
  QString dir_path = file_info.dir().path();
  QDir::setCurrent(dir_path);

  project.building.clear();
  create_scene();
  file_save_project();
  level_table->update(project.building);

  QSettings settings;
  settings.setValue(
      preferences_keys::previous_project_path,
      QString::fromStdString(project.filename));
}

void Editor::file_open_project()
{
  QFileDialog file_dialog(this, "Open Project");
  file_dialog.setFileMode(QFileDialog::ExistingFile);
  file_dialog.setNameFilter("*.yaml");

  if (file_dialog.exec() != QDialog::Accepted)
    return;
  
  QFileInfo file_info(file_dialog.selectedFiles().first());
  if (!file_info.exists()) {
    QMessageBox::critical(
        this,
        "File does not exist",
        "File does not exist. Cannot open file.");
    return;
  }
  load_project(file_info.filePath());
}

bool Editor::file_save_project()
{
  project.save_yaml_file();
  setWindowModified(false);
  return true;
}

void Editor::help_about()
{
  QMessageBox::about(this, "About", "Welcome to the Traffic Editor");
}

void Editor::edit_preferences()
{
  PreferencesDialog preferences_dialog(this);

  if (preferences_dialog.exec() == QDialog::Accepted)
    load_model_names();
}

void Editor::edit_building_properties()
{
  BuildingDialog building_dialog(project.building);
  if (building_dialog.exec() == QDialog::Accepted)
    setWindowModified(true);
}

void Editor::zoom_fit()
{
  // todo: implement this for real
  //map_view->set_absolute_scale(1.0);
  map_view->resetMatrix();
}

void Editor::mouse_event(const MouseType t, QMouseEvent *e)
{
  QPointF p;
  if (!is_mouse_event_in_map(e, p)) {
    e->ignore();
    return;
  }
  if (level_idx >= static_cast<int>(project.building.levels.size())) {
    if (t == MOUSE_RELEASE) {
      if (project.filename.empty()) {
        QMessageBox::critical(
            this,
            "No project",
            "Please try File->New Project... or File->Open Project...");
      }
      else if (project.building.levels.empty()) {
        QMessageBox::critical(
            this,
            "No levels defined",
            "No levels defined. Use Level->Add...");
      }
    }
    return;
  }
  // dispatch to individual mouse handler functions to save indenting...
  switch (tool_id) {
    case TOOL_SELECT:       mouse_select(t, e, p); break;
    case TOOL_ADD_VERTEX:   mouse_add_vertex(t, e, p); break;
    case TOOL_MOVE:         mouse_move(t, e, p); break;
    case TOOL_ADD_LANE:     mouse_add_lane(t, e, p); break;
    case TOOL_ADD_WALL:     mouse_add_wall(t, e, p); break;
    case TOOL_ADD_MEAS:     mouse_add_meas(t, e, p); break;
    case TOOL_ADD_DOOR:     mouse_add_door(t, e, p); break;
    case TOOL_ADD_MODEL:    mouse_add_model(t, e, p); break;
    case TOOL_ROTATE:       mouse_rotate(t, e, p); break;
    case TOOL_ADD_FLOOR:    mouse_add_floor(t, e, p); break;
    case TOOL_EDIT_POLYGON: mouse_edit_polygon(t, e, p); break;
    case TOOL_ADD_FIDUCIAL: mouse_add_fiducial(t, e, p); break;
    default: break;
  }
  previous_mouse_point = p;
}

void Editor::mousePressEvent(QMouseEvent *e)
{
  mouse_event(MOUSE_PRESS, e);
}

void Editor::mouseReleaseEvent(QMouseEvent *e)
{
  mouse_event(MOUSE_RELEASE, e);
}

void Editor::mouseMoveEvent(QMouseEvent *e)
{
  mouse_event(MOUSE_MOVE, e);
}

int Editor::get_polygon_idx(const double x, const double y)
{
  const Level &level = project.building.levels[level_idx];
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
      if (project.building.delete_selected(level_idx))
      {
        clear_property_editor();
        setWindowModified(true);
      }
      else
      {
        QMessageBox::critical(
            this,
            "Could not delete item",
            "If deleting a vertex, it must not be in any edges or polygons.");

        clear_selection();
      }
      create_scene();
      break;
    case Qt::Key_S:
    case Qt::Key_Escape:
      tool_button_group->button(TOOL_SELECT)->click();
      clear_selection();
      update_property_editor();
      create_scene();
      break;
    case Qt::Key_V:
      tool_button_group->button(TOOL_ADD_VERTEX)->click();
      break;
    case Qt::Key_M:
      tool_button_group->button(TOOL_MOVE)->click();
      break;
    case Qt::Key_L:
      tool_button_group->button(TOOL_ADD_LANE)->click();
      break;
    case Qt::Key_W:
      tool_button_group->button(TOOL_ADD_WALL)->click();
      break;
    case Qt::Key_T:
      tool_button_group->button(TOOL_ADD_MEAS)->click();
      break;
    case Qt::Key_O:
      tool_button_group->button(TOOL_ADD_MODEL)->click();
      break;
    case Qt::Key_R:
      tool_button_group->button(TOOL_ROTATE)->click();
      break;
    case Qt::Key_F:
      tool_button_group->button(TOOL_ADD_FLOOR)->click();
      break;
    case Qt::Key_E:
      tool_button_group->button(TOOL_EDIT_POLYGON)->click();
      break;
    case Qt::Key_B:
      for (auto &edge : project.building.levels[level_idx].edges) {
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
  switch (id)
  {
    case TOOL_SELECT: return "&select";
    case TOOL_MOVE: return "&move";
    case TOOL_ROTATE: return "&rotate";
    case TOOL_ADD_VERTEX: return "add &vertex";
    case TOOL_ADD_LANE: return "add &lane";
    case TOOL_ADD_WALL: return "add &wall";
    case TOOL_ADD_MEAS: return "add measuremen&t";
    case TOOL_ADD_DOOR: return "add door";
    case TOOL_ADD_MODEL: return "add m&odel";
    case TOOL_ADD_FLOOR: return "add &floor";
    case TOOL_EDIT_POLYGON: return "&edit polygon";
    default: return "unknown tool ID";
  }
}

void Editor::tool_toggled(int id, bool checked)
{
  if (!checked)
    return;

  clicked_idx = -1;
  remove_mouse_motion_item();

  tool_id = static_cast<ToolId>(id);

#if 0
  // TODO: need to improve logic to set back to "normal" cursor...
  // set the cursor
  Qt::CursorShape cursor = Qt::ArrowCursor;
  if (tool_id == TOOL_ADD_VERTEX)
    cursor = Qt::CrossCursor;
  map_view->setCursor(cursor);
#endif

  // set the status bar
  switch (tool_id)
  {
    case TOOL_SELECT:
      statusBar()->showMessage("Click an item to select it.");
      break;
    case TOOL_ADD_LANE:
    case TOOL_ADD_WALL:
    case TOOL_ADD_MEAS:
      statusBar()->showMessage(
          "Click and drag from a vertex to another vertex to add an edge.");
      break;
    case TOOL_ADD_FLOOR:
      statusBar()->showMessage(
          "Left-click to add polygon vertices. "
          "Right-click to close polygon.");
      break;
    case TOOL_EDIT_POLYGON:
      statusBar()->showMessage(
          "Left-click drag an edge to introduce a new vertex. "
          "Right-click a polygon vertex to remove it from the polygon.");
      break;
    default:
      statusBar()->clearMessage();
      break;
  }

  // execute dialogs as needed
  if (tool_id == TOOL_ADD_MODEL)
  {
    Model model;
    ModelDialog dialog(this, model, editor_models);
    if (dialog.exec() == QDialog::Accepted)
    {
      // find the EditorModel with the requested name
      for (auto& em : editor_models)
        if (em.name == model.model_name)
        {
          mouse_motion_editor_model = &em;
          const QPixmap pixmap(mouse_motion_editor_model->get_pixmap());
          mouse_motion_model = scene->addPixmap(pixmap);
          mouse_motion_model->setOffset(-pixmap.width()/2, -pixmap.height()/2);
          mouse_motion_model->setScale(
              mouse_motion_editor_model->meters_per_pixel /
              project.building.levels[level_idx].drawing_meters_per_pixel);
          mouse_motion_model->setPos(
              previous_mouse_point.x(),
              previous_mouse_point.y());
          statusBar()->showMessage("Left-click to instantiate this model.");
          break;
        }
    }
    else
      tool_button_group->button(TOOL_SELECT)->click();  // back to select mode
  }
}

void Editor::update_property_editor()
{
  add_param_button->setEnabled(false);
  delete_param_button->setEnabled(false);

  if (project.building.levels.empty())
    return;

  for (const auto& p : project.building.levels[level_idx].polygons)
    if (p.selected)
    {
      printf("found a selected polygon\n");
      // todo: populate property editor
      return;
    }

  for (const auto& e : project.building.levels[level_idx].edges)
    if (e.selected)
    {
      populate_property_editor(e);
      return;  // stop after finding the first one
    }

  for (const auto& m : project.building.levels[level_idx].models)
    if (m.selected)
    {
      populate_property_editor(m);
      return;  // stop after finding the first one
    }

  for (const auto& v : project.building.levels[level_idx].vertices)
    if (v.selected)
    {
      populate_property_editor(v);
      return;  // stop after finding the first one
    }

  for (const auto& f : project.building.levels[level_idx].fiducials)
    if (f.selected)
    {
      populate_property_editor(f);
      return;  // stop after finding the first one
    }

  // if we get here, we never found anything :(
  clear_property_editor();
}

QTableWidgetItem *Editor::create_table_item(
    const QString &str,
    bool editable)
{
  QTableWidgetItem *item = new QTableWidgetItem(str);
  if (!editable)
    item->setFlags(Qt::NoItemFlags);
  else
    item->setBackground(QBrush(Qt::white));
  return item;
}

void Editor::property_editor_set_row(
    const int row_idx,
    const QString &label,
    const QString &value,
    const bool editable)
{
  QTableWidgetItem *label_item = new QTableWidgetItem(label);
  label_item->setFlags(Qt::NoItemFlags);

  QTableWidgetItem *value_item = new QTableWidgetItem(value);
  if (!editable)
    value_item->setFlags(Qt::NoItemFlags);
  else
    value_item->setBackground(QBrush(Qt::white));

  property_editor->setItem(row_idx, 0, label_item);
  property_editor->setItem(row_idx, 1, value_item);
}

void Editor::property_editor_set_row(
    const int row_idx,
    const QString &label,
    const int &value,
    const bool editable)
{
  property_editor_set_row(row_idx, label, QString::number(value), editable);
}

void Editor::property_editor_set_row(
    const int row_idx,
    const QString &label,
    const double &value,
    const int num_decimal_places,
    const bool editable)
{
  property_editor_set_row(
      row_idx,
      label,
      QString::number(value, 'g', num_decimal_places + 1),
      editable);
}

void Editor::add_param_button_clicked()
{
  const string object_type =
      add_param_button->property("object_type").toString().toStdString();
  printf("add param object type: %s\n", object_type.c_str());

  if (object_type == "vertex")
  {
    AddParamDialog dialog(this, Vertex::allowed_params);
    if (dialog.exec() != QDialog::Accepted)
      return;

    for (auto &v : project.building.levels[level_idx].vertices)
    {
      if (v.selected)
      {
        v.params[dialog.get_param_name()] = Param(dialog.get_param_type());
        populate_property_editor(v);
        setWindowModified(true);
        return;  // stop after finding the first one
      }
    }
  }
}

void Editor::delete_param_button_clicked()
{
  QMessageBox::about(
      this,
      "work in progress",
      "TODO: something...sorry. For now, hand-edit the YAML.");
}

void Editor::populate_layers_table()
{
  if (project.building.levels.empty())
    return;  // let's not crash...
  const Level &level = project.building.levels[level_idx];
  layers_table->blockSignals(true);  // otherwise we get tons of callbacks
  layers_table->setRowCount(2 + level.layers.size());

  layers_table->blockSignals(true);  // otherwise we get tons of callbacks
  layers_table_set_row(0, "floorplan", true);

  for (size_t i = 0; i < level.layers.size(); i++)
  {
    layers_table_set_row(
        i + 1,
        QString::fromStdString(level.layers[i].name),
        level.layers[i].visible);
  }

  const int last_row_idx = static_cast<int>(level.layers.size()) + 1;
  // we'll use the last row for the "Add" button
  layers_table->setCellWidget(last_row_idx, 0, nullptr);
  QPushButton *add_button = new QPushButton("Add...", this);
  layers_table->setCellWidget(last_row_idx, 1, add_button);
  connect(
      add_button, &QAbstractButton::clicked,
      [=]() { this->layer_add_button_clicked(); });

  layers_table->blockSignals(false);  // re-enable callbacks
}

void Editor::layers_table_set_row(
    const int row_idx,
    const QString &label,
    const bool checked)
{
  QCheckBox *checkbox = new QCheckBox(label);
  checkbox->setChecked(checked);
  layers_table->setCellWidget(row_idx, 0, checkbox);

  QPushButton *button = new QPushButton("Edit...", this);
  layers_table->setCellWidget(row_idx, 1, button);

  connect(
      button, &QAbstractButton::clicked,
      [=]() { this->layer_edit_button_clicked(label.toStdString()); });
  connect(
      checkbox, &QAbstractButton::clicked,
      [=](bool box_checked)
      {
        if (row_idx > 0)
          project.building.levels[level_idx].layers[row_idx-1].visible =
              box_checked;
        create_scene();
      });
}

void Editor::layer_edit_button_clicked(const std::string &label)
{
  printf("clicked: [%s]\n", label.c_str());
  if (project.building.levels.empty())
    return;
  // find the index of this layer in the current level
  Level &level = project.building.levels[level_idx];
  for (size_t i = 0; i < level.layers.size(); i++)
  {
    Layer& layer = level.layers[i];
    if (label != layer.name)
      continue;
    LayerDialog *dialog = new LayerDialog(this, layer, true);
    dialog->show();
    dialog->raise();
    dialog->activateWindow();
    connect(
        dialog,
        &LayerDialog::redraw,
        this,
        &Editor::create_scene);
    return;  // only create a dialog for the first name match
  }
}

void Editor::layer_add_button_clicked()
{
  if (level_idx >= static_cast<int>(project.building.levels.size()))
    return;  // let's not crash (yet)
  Level& level = project.building.levels[level_idx];
  Layer layer;
  LayerDialog layer_dialog(this, layer);
  if (layer_dialog.exec() != QDialog::Accepted)
    return;
  printf("added a layer: [%s]\n", layer.name.c_str());
  level.layers.push_back(layer);
  populate_layers_table();
  setWindowModified(true);
}

void Editor::populate_property_editor(const Edge& edge)
{
  const BuildingLevel& level = project.building.levels[level_idx];
  const double scale = level.drawing_meters_per_pixel;
  const Vertex &sv = level.vertices[edge.start_idx];
  const Vertex &ev = level.vertices[edge.end_idx];

  const double sx = sv.x * scale;
  const double sy = sv.y * scale;
  const double ex = ev.x * scale;
  const double ey = ev.y * scale;

  const double dx = ex - sx;
  const double dy = ey - sy;
  const double len = sqrt(dx*dx + dy*dy);

  property_editor->blockSignals(true);  // otherwise we get tons of callbacks
  property_editor->setRowCount(8 + edge.params.size());

  property_editor_set_row(0, "edge_type", edge.type_to_qstring());
  property_editor_set_row(1, "start_idx", edge.start_idx);
  property_editor_set_row(2, "end_idx", edge.end_idx);
  property_editor_set_row(3, "start x (m)", sx);
  property_editor_set_row(4, "start y (m)", sy);
  property_editor_set_row(5, "end x (m)", ex);
  property_editor_set_row(6, "end y (m)", ey);
  property_editor_set_row(7, "length (m)", len);

  int row = 8;
  for (const auto &param : edge.params) {
    property_editor_set_row(
        row,
        QString::fromStdString(param.first),
        param.second.to_qstring(),
        true);
    row++;
  }

  property_editor->blockSignals(false);  // re-enable callbacks
}

void Editor::populate_property_editor(const Vertex& vertex)
{
  const BuildingLevel& level = project.building.levels[level_idx];
  const double scale = level.drawing_meters_per_pixel;
 
  property_editor->blockSignals(true);  // otherwise we get tons of callbacks
  property_editor->setRowCount(5 + vertex.params.size());

  property_editor_set_row(0, "x (pixels)", vertex.x);
  property_editor_set_row(1, "y (pixels)", vertex.y);
  property_editor_set_row(2, "x (m)", vertex.x * scale);
  property_editor_set_row(3, "y (m)", -1.0 * vertex.y * scale);
  property_editor_set_row(
      4,
      "name",
      QString::fromStdString(vertex.name),
      true);

  int row = 5;
  for (const auto &param : vertex.params) {
    property_editor_set_row(
        row,
        QString::fromStdString(param.first),
        param.second.to_qstring(),
        true);
    row++;
  }

  add_param_button->setEnabled(true);
  add_param_button->setProperty("object_type", QVariant("vertex"));

  property_editor->blockSignals(false);  // re-enable callbacks
}

void Editor::populate_property_editor(const Fiducial& fiducial)
{
  property_editor->blockSignals(true);

  property_editor->setRowCount(1);
  property_editor_set_row(
      0,
      "name",
      QString::fromStdString(fiducial.name),
      true);  // true means that this cell value is editable

  property_editor->blockSignals(false);
}

void Editor::populate_property_editor(const Model& model)
{
  printf("populate_property_editor(model)\n");
  property_editor->blockSignals(true);  // otherwise we get tons of callbacks

  property_editor->setRowCount(2);

  property_editor_set_row(
      0,
      "name",
      QString::fromStdString(model.instance_name));

  property_editor_set_row(
      1,
      "model_name",
      QString::fromStdString(model.model_name));
      
  property_editor->blockSignals(false);  // re-enable callbacks
}

void Editor::clear_property_editor()
{
  property_editor->setRowCount(0);
  add_param_button->setEnabled(false);
  delete_param_button->setEnabled(false);
}

void Editor::property_editor_cell_changed(int row, int column)
{
  std::string name = property_editor->item(row, 0)->text().toStdString();
  std::string value = property_editor->item(row, 1)->text().toStdString();
  printf("property_editor_cell_changed(%d, %d) = param %s\n",
      row, column, name.c_str());

  for (auto& v : project.building.levels[level_idx].vertices)
  {
    if (!v.selected)
      continue;
    if (name == "name")
      v.name = value;
    else
      v.set_param(name, value);
    create_scene();
    setWindowModified(true);
    return;  // stop after finding the first one
  }

  for (auto& e : project.building.levels[level_idx].edges)
  {
    if (!e.selected)
      continue;
    e.set_param(name, value);
    create_scene();
    setWindowModified(true);
    return;  // stop after finding the first one
  }

  for (auto& f : project.building.levels[level_idx].fiducials)
  {
    if (!f.selected)
      continue;
    if (name == "name")
      f.name = value;
    create_scene();
    setWindowModified(true);
    return;  // stop after finding the first one
  }
}

bool Editor::create_scene()
{
  scene->clear();  // destroys the mouse_motion_* items if they are there
  mouse_motion_line = nullptr;
  mouse_motion_model = nullptr;
  mouse_motion_ellipse = nullptr;
  mouse_motion_polygon = nullptr;

  if (project.building.levels.empty())
  {
    printf("nothing to draw!\n");
    return false;
  }

  project.building.levels[level_idx].draw(scene, editor_models);
  project.building.draw_lifts(scene, level_idx);

  return true;
}

void Editor::clear_selection()
{
  if (project.building.levels.empty())
    return;
  project.building.levels[level_idx].clear_selection();
}

void Editor::draw_mouse_motion_line_item(
    const double mouse_x,
    const double mouse_y)
{
  double pen_width = 1;
  QColor color;
  switch (tool_id) {
    case TOOL_ADD_LANE:
      pen_width = 20;
      color = QColor::fromRgbF(0, 0, 1, 0.5); 
      break;
    case TOOL_ADD_WALL:
      pen_width = 5;
      color = QColor::fromRgbF(0, 0, 1, 0.5); 
      break;
    case TOOL_ADD_MEAS:
      pen_width = 5;
      color = QColor::fromRgbF(1, 0, 1, 0.5); 
      break;
    default:
      break;
  }

  QPen pen(QBrush(color), pen_width, Qt::SolidLine, Qt::RoundCap);
  const auto& start = project.building.levels[level_idx].vertices[clicked_idx];
  if (!mouse_motion_line)
    mouse_motion_line = scene->addLine(start.x, start.y, mouse_x, mouse_y, pen);
  else
    mouse_motion_line->setLine(start.x, start.y, mouse_x, mouse_y);
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
  mouse_motion_editor_model = nullptr;

  mouse_vertex_idx = -1;
  mouse_fiducial_idx = -1;
}

void Editor::set_selected_line_item(QGraphicsLineItem *line_item)
{
  clear_selection();

  if (line_item == nullptr)
    return;

  // find if any of our lanes match those vertices
  for (auto& edge : project.building.levels[level_idx].edges)
  {
    const auto& v_start = project.building.levels[level_idx].vertices[edge.start_idx];
    const auto& v_end = project.building.levels[level_idx].vertices[edge.end_idx];
    if (line_vertices_match(line_item, v_start, v_end, 10.0))
    {
      edge.selected = true;
      return;  // stop after first one is found, don't select multiple
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
  if (type != MOUSE_PRESS)
    return;
  clear_selection();

  Building::NearestItem ni = project.building.nearest_items(level_idx, p.x(), p.y());

  if (ni.model_idx >= 0 && ni.model_dist < 50.0)
    project.building.levels[level_idx].models[ni.model_idx].selected = true;
  else if (ni.vertex_idx >= 0 && ni.vertex_dist < 10.0)
    project.building.levels[level_idx].vertices[ni.vertex_idx].selected = true;
  else if (ni.fiducial_idx >= 0 && ni.fiducial_dist < 10.0)
    project.building.levels[level_idx].fiducials[ni.fiducial_idx].selected = true;
  else
  {
    const QPoint p_global = mapToGlobal(e->pos());
    const QPoint p_map = map_view->mapFromGlobal(p_global);

    // use the QGraphics stuff to see if it's an edge segment or polygon
    QGraphicsItem *item = map_view->itemAt(p_map);
    if (item)
    {
      switch (item->type())
      {
        case QGraphicsLineItem::Type:
          printf("clicked line\n");
          set_selected_line_item(
              qgraphicsitem_cast<QGraphicsLineItem *>(item));
          break;
  
        case QGraphicsPolygonItem::Type:
        { // need new scope due to 'for' iterator variable
          printf("clicked polygon\n");
          polygon_idx = get_polygon_idx(p.x(), p.y());
          if (polygon_idx < 0)
            return;  // didn't click on a polygon
          Polygon &polygon = project.building.levels[level_idx].polygons[polygon_idx];
          polygon.selected = true;
          for (const auto &vertex_idx : polygon.vertices)
            project.building.levels[level_idx].vertices[vertex_idx].selected = true;
          break;
        }
  
        default:
          printf("clicked unhandled type: %d\n",
              static_cast<int>(item->type()));
          break;
      }
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
  if (t == MOUSE_PRESS) {
    project.building.add_vertex(level_idx, p.x(), p.y());
    setWindowModified(true);
    create_scene();
  }
}

void Editor::mouse_add_fiducial(
    const MouseType t, QMouseEvent *, const QPointF &p)
{
  if (t == MOUSE_PRESS)
  {
    project.building.add_fiducial(level_idx, p.x(), p.y());
    setWindowModified(true);
    create_scene();
  }
}

void Editor::mouse_move(
    const MouseType t, QMouseEvent *e, const QPointF &p)
{
  if (t == MOUSE_PRESS)
  {
    Building::NearestItem ni = project.building.nearest_items(level_idx, p.x(), p.y());

    if (ni.model_idx >= 0 && ni.model_dist < 50.0)
    {
      // Now we need to find the pixmap item for this model.
      mouse_motion_model = get_closest_pixmap_item(
          QPointF(
              project.building.levels[level_idx].models[clicked_idx].x,
              project.building.levels[level_idx].models[clicked_idx].y));
      mouse_model_idx = ni.model_idx;
    }
    else if (ni.vertex_idx >= 0 && ni.vertex_dist < 10.0)
    {
      mouse_vertex_idx = ni.vertex_idx;
      // todo: save the QGrahpicsEllipse or group, to avoid full repaints?
    }
    else if (ni.fiducial_idx >= 0 && ni.fiducial_dist < 10.0)
    {
      mouse_fiducial_idx = ni.fiducial_idx;
      // todo: save the QGrahpicsEllipse or group, to avoid full repaints?
    }
  }
  else if (t == MOUSE_RELEASE)
  {
    mouse_vertex_idx = -1;
    mouse_fiducial_idx = -1;
    create_scene();  // this will free mouse_motion_model
    setWindowModified(true);
  }
  else if (t == MOUSE_MOVE)
  {
    if (!(e->buttons() & Qt::LeftButton))
      return;  // we only care about mouse-dragging, not just motion
    printf("mouse move, vertex_idx = %d, fiducial_idx = %d\n",
        mouse_vertex_idx,
        mouse_fiducial_idx);
    if (mouse_motion_model != nullptr)
    {
      // we're dragging a model
      // update both the nav_model data and the pixmap in the scene
      project.building.levels[level_idx].models[mouse_model_idx].x = p.x();
      project.building.levels[level_idx].models[mouse_model_idx].y = p.y();
      mouse_motion_model->setPos(p);
    }
    else if (mouse_vertex_idx >= 0)
    {
      // we're dragging a vertex
      Vertex& pt = project.building.levels[level_idx].vertices[mouse_vertex_idx];
      pt.x = p.x();
      pt.y = p.y();
      create_scene();
    }
    else if (mouse_fiducial_idx >= 0)
    {
      Fiducial& f = project.building.levels[level_idx].fiducials[mouse_fiducial_idx];
      f.x = p.x();
      f.y = p.y();
      printf("moved fiducial %d to (%.1f, %.1f)\n",
          mouse_fiducial_idx,
          f.x,
          f.y);
      create_scene();
    }
  }
}

void Editor::mouse_add_edge(
    const MouseType t,
    QMouseEvent *,
    const QPointF &p,
    const Edge::Type &edge_type)
{
  if (t == MOUSE_PRESS)
  {
    clicked_idx = project.building.nearest_item_index_if_within_distance(
        level_idx, p.x(), p.y(), 10.0, Building::VERTEX);
  }
  else if (t == MOUSE_RELEASE)
  {
    if (clicked_idx < 0)
      return;
    remove_mouse_motion_item();
    double distance = 0;
    const int release_idx = project.building.find_nearest_vertex_index(
        level_idx, p.x(), p.y(), distance);
    if (distance > 10.0 || (clicked_idx == release_idx)) {
      clicked_idx = -1;
      return;
    }
    project.building.add_edge(level_idx, clicked_idx, release_idx, edge_type);
    clicked_idx = -1;
    setWindowModified(true);
    create_scene();
  }
  else if (t == MOUSE_MOVE)
  {
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

void Editor::mouse_add_door(
    const MouseType t, QMouseEvent *e, const QPointF &p)
{
  mouse_add_edge(t, e, p, Edge::DOOR);
}

void Editor::mouse_add_model(
    const MouseType t, QMouseEvent *, const QPointF &p)
{
  if (t == MOUSE_PRESS)
  {
    if (mouse_motion_editor_model == nullptr)
      return;
    project.building.add_model(
        level_idx,
        p.x(),
        p.y(),
        0.0,
        mouse_motion_editor_model->name);
    /*
    const int model_row = model_name_list_widget->currentRow();
    if (model_row < 0)
      return;  // nothing currently selected. nothing to do.
    .add_model(level_idx, p.x(), p.y(), 0.0, editor_models[model_row].name);
    */
    setWindowModified(true);
    create_scene();
  }
  else if (t == MOUSE_MOVE)
  {
    if (mouse_motion_editor_model == nullptr)
      return;  // nothing currently selected. nothing to do.
    if (mouse_motion_model == nullptr)
    {
      const QPixmap pixmap(mouse_motion_editor_model->get_pixmap());
      mouse_motion_model = scene->addPixmap(pixmap);
      mouse_motion_model->setOffset(-pixmap.width()/2, -pixmap.height()/2);
      mouse_motion_model->setScale(
          mouse_motion_editor_model->meters_per_pixel /
          project.building.levels[level_idx].drawing_meters_per_pixel);
    }
    mouse_motion_model->setPos(p.x(), p.y());
  }
}

double Editor::discretize_angle(const double &angle)
{
  const double discretization = 45.0 * M_PI / 180.0;
  return discretization * round(angle / discretization);
}

void Editor::mouse_rotate(
    const MouseType t, QMouseEvent *mouse_event, const QPointF &p)
{
  if (t == MOUSE_PRESS) {
    clicked_idx = project.building.nearest_item_index_if_within_distance(
        level_idx,
        p.x(),
        p.y(),
        50.0,
        Building::MODEL);
    if (clicked_idx < 0)
      return; // nothing to do. click wasn't on a model.

    const Model &model = project.building.levels[level_idx].models[clicked_idx];
    mouse_motion_model = get_closest_pixmap_item(
        QPointF(model.x, model.y));
  
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
        model.y - r * sin(model.yaw),
        pen);
  }
  else if (t == MOUSE_RELEASE) {
    //remove_mouse_motion_item();
    if (clicked_idx < 0)
      return;
    const Model &model = project.building.levels[level_idx].models[clicked_idx];
    const double dx = p.x() - model.x;
    const double dy = -(p.y() - model.y);  // vertical axis is flipped
    double mouse_yaw = atan2(dy, dx);
    if (mouse_event->modifiers() & Qt::ShiftModifier)
      mouse_yaw = discretize_angle(mouse_yaw);
    project.building.set_model_yaw(level_idx, clicked_idx, mouse_yaw);
    clicked_idx = -1;  // we're done rotating it now
    setWindowModified(true);
    // now re-render the whole scene (could optimize in the future...)
    create_scene();
  }
  else if (t == MOUSE_MOVE) {
    if (clicked_idx < 0)
      return;  // nothing currently selected. nothing to do.

    // re-orient the mouse_motion_model item and heading indicator as needed
    const Model &model = project.building.levels[level_idx].models[clicked_idx];
    const double dx = p.x() - model.x;
    const double dy = -(p.y() - model.y);  // vertical axis is flipped
    double mouse_yaw = atan2(dy, dx);
    if (mouse_event->modifiers() & Qt::ShiftModifier)
      mouse_yaw = discretize_angle(mouse_yaw);
    const double r = static_cast<double>(ROTATION_INDICATOR_RADIUS);
    mouse_motion_line->setLine(
        model.x,
        model.y,
        model.x + r * cos(mouse_yaw),
        model.y - r * sin(mouse_yaw));

    if (mouse_motion_model)
      mouse_motion_model->setRotation(-mouse_yaw * 180.0 / M_PI);
  }
}

QGraphicsPixmapItem *Editor::get_closest_pixmap_item(const QPointF &p)
{
  // todo: use fancier calls if the scene graph gets so big that a linear
  // search becomes intolerably slow
  const QList <QGraphicsItem *> items = scene->items();
  QGraphicsPixmapItem *pixmap_item = nullptr;
  double min_dist = 1.0e9;
  for (const auto item : items) {
    if (item->type() != QGraphicsPixmapItem::Type)
      continue;  // ignore anything other than the pixmaps (models)
    const double model_click_distance = QLineF(p, item->pos()).length();
    if (model_click_distance < min_dist) {
      min_dist = model_click_distance;
      pixmap_item = qgraphicsitem_cast<QGraphicsPixmapItem *>(item);
    }
  }
  return pixmap_item;
}

void Editor::mouse_add_polygon(
    const MouseType t,
    QMouseEvent *e,
    const QPointF &p,
    const Polygon::Type &polygon_type)
{
  if (t == MOUSE_PRESS) {
    if (e->buttons() & Qt::LeftButton) {
      clicked_idx = project.building.nearest_item_index_if_within_distance(
          level_idx, p.x(), p.y(), 10.0, Building::VERTEX);
      if (clicked_idx < 0)
        return; // nothing to do. click wasn't on a vertex.

      Vertex &v = project.building.levels[level_idx].vertices[clicked_idx];
      v.selected = true;  // todo: find graphics item for vertex and colorize it
    
      if (mouse_motion_polygon == nullptr) {
        QVector<QPointF> polygon_vertices;
        polygon_vertices.append(QPointF(v.x, v.y));
        QPolygonF polygon(polygon_vertices);
        mouse_motion_polygon = scene->addPolygon(
            polygon,
            QPen(Qt::black),
            QBrush(QColor::fromRgbF(1.0, 0.0, 0.0, 0.5)));
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
        project.building.levels[level_idx].polygons.push_back(polygon);
      }
      scene->removeItem(mouse_motion_polygon);
      delete mouse_motion_polygon;
      mouse_motion_polygon = nullptr;

      setWindowModified(true);
      clear_selection();
      create_scene();
    }
  }
  else if (t == MOUSE_MOVE) {
    if (mouse_motion_polygon == nullptr)
      return;  // not sure how we got here

    // first, remove the previous polygon
    scene->removeItem(mouse_motion_polygon);
    delete mouse_motion_polygon;
 
    // now, make the updated polygon
    QVector<QPointF> polygon_vertices;
    for (const auto &vertex_idx: mouse_motion_polygon_vertices) {
      const Vertex &v = project.building.levels[level_idx].vertices[vertex_idx];
      polygon_vertices.append(QPointF(v.x, v.y));
    }
    polygon_vertices.append(QPointF(p.x(), p.y()));
 
    // insert the updated polygon into the scene
    QPolygonF polygon(polygon_vertices);
    mouse_motion_polygon = scene->addPolygon(
        polygon,
        QPen(Qt::black),
        QBrush(QColor::fromRgbF(1.0, 0.0, 0.0, 0.5)));
  }
}

void Editor::mouse_add_floor(
    const MouseType t, QMouseEvent *e, const QPointF &p)
{
  mouse_add_polygon(t, e, p, Polygon::FLOOR);
}

void Editor::mouse_edit_polygon(
    const MouseType t, QMouseEvent *e, const QPointF &p)
{
  if (t == MOUSE_PRESS) {
    if (e->buttons() & Qt::RightButton) {
      if (polygon_idx < 0)
        return;  // no polygon is selected, nothing to do
      int vertex_idx = project.building.nearest_item_index_if_within_distance(
          level_idx, p.x(), p.y(), 10.0, Building::VERTEX);
      if (vertex_idx < 0)
        return;  // Nothing to do. Click wasn't near a vertex.
      // first mark the vertex as no longer selected
      project.building.remove_polygon_vertex(level_idx, polygon_idx, vertex_idx);
      setWindowModified(true);
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
          project.building.polygon_edge_drag_press(level_idx, polygon_idx, x, y);
      if (polygon_vertex_drag_idx < 0)
        return;
    
      if (mouse_motion_polygon != nullptr) {
        qWarning("edit_polygon_release() without null mouse_motion_polygon!");
        return;
      }
    
      // create the mouse motion polygon and insert a new edge
      QVector<QPointF> polygon_vertices;
      const Polygon &polygon =
          project.building.levels[level_idx].polygons[polygon_idx];
      for (size_t i = 0; i < polygon.vertices.size(); i++) {
        const int v_idx = polygon.vertices[i];
        const Vertex &v = project.building.levels[level_idx].vertices[v_idx];
        polygon_vertices.append(QPointF(v.x, v.y));
        if (v_idx == polygon_vertex_drag_idx) {
          polygon_vertices.append(QPointF(x, y));  // current mouse location
          mouse_motion_polygon_vertex_idx = i + 1;
        }
      }
      QPolygonF drag_polygon(polygon_vertices);
    
      mouse_motion_polygon = scene->addPolygon(
          drag_polygon,
          QPen(Qt::black),
          QBrush(QColor::fromRgbF(1.0, 1.0, 0.5, 0.5)));
    }
  }
  else if (t == MOUSE_RELEASE) {
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
  
    int release_vertex_idx = project.building.nearest_item_index_if_within_distance(
        level_idx, p.x(), p.y(), 10.0, Building::VERTEX);
    if (release_vertex_idx < 0)
      return;  // nothing to do; didn't release near a vertex
  
    Polygon &existing = project.building.levels[level_idx].polygons[polygon_idx];
    if (std::find(
        existing.vertices.begin(),
        existing.vertices.end(),
        release_vertex_idx) != existing.vertices.end())
      return;  // Release vertex is already in the polygon. Don't do anything.
  
    existing.vertices.insert(
        existing.vertices.begin() + mouse_motion_polygon_vertex_idx,
        release_vertex_idx);
  
    setWindowModified(true);
    create_scene();
  }
  else if (t == MOUSE_MOVE) {
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

void Editor::number_key_pressed(const int n)
{
  for (auto &edge : project.building.levels[level_idx].edges)
  {
    if (edge.selected && edge.type == Edge::LANE)
      edge.set_graph_idx(n);
  }
  create_scene();
  update_property_editor();
}

bool Editor::maybe_save()
{
  if (!isWindowModified())
    return true;  // no need to ask to save the document
  const QMessageBox::StandardButton button_clicked =
      QMessageBox::warning(
          this,
          "Project not saved!",
          "Do you want to save your changes?",
          QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
  switch (button_clicked)
  {
    case QMessageBox::Save:
      return file_save_project();
    case QMessageBox::Cancel:
      return false;
    default:
      break;
  }
  return true;
}

void Editor::closeEvent(QCloseEvent *event)
{
  // save window geometry
  QSettings settings;
  settings.setValue(preferences_keys::left, geometry().x());
  settings.setValue(preferences_keys::top, geometry().y());
  settings.setValue(preferences_keys::width, geometry().width());
  settings.setValue(preferences_keys::height, geometry().height());

  if (maybe_save())
    event->accept();
  else
    event->ignore();
}

void Editor::mode_building()
{
  if (mode_combo_box->currentText() != "Building")
    mode_combo_box->setCurrentText("Building");
  printf("mode_building()\n");
}

void Editor::mode_traffic()
{
  if (mode_combo_box->currentText() != "Traffic")
    mode_combo_box->setCurrentText("Traffic");
  printf("mode_traffic()\n");
}
