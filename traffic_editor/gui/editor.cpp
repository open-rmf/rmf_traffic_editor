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
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include <QtWidgets>

#include <QInputDialog>
#include <QLabel>
#include <QListWidget>
#include <QToolBar>

#ifdef HAS_OPENCV
#include <opencv2/core/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#endif

#include <yaml-cpp/yaml.h>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_resource.hpp"

#include "actions/add_fiducial.h"
#include "actions/add_model.h"
#include "actions/add_property.h"
#include "actions/add_polygon.h"
#include "actions/add_vertex.h"
#include "actions/delete.h"
#include "actions/polygon_add_vertex.h"
#include "actions/polygon_remove_vertices.h"

#include "add_param_dialog.h"
#include "building_dialog.h"
#include "building_level_dialog.h"
#include "building_level_table.h"
#include "editor.h"
#include "layer_dialog.h"
#include "lift_table.h"
#include "map_view.h"
#include "model_dialog.h"
#include "preferences_dialog.h"
#include "preferences_keys.h"
#include "project_dialog.h"
#include "scenario_table.h"
#include "traffic_table.h"
#include "ui_transform_dialog.h"


using std::string;
using std::isnan;


Editor* Editor::instance = nullptr;

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

  QVBoxLayout* left_layout = new QVBoxLayout;
  left_layout->addWidget(map_view);

  layers_table = new TableList;  // todo: replace with specific subclass?

  level_table = new BuildingLevelTable;
  connect(
    level_table, &QTableWidget::cellClicked,
    [=](int row, int /*col*/)
    {
      if (row < static_cast<int>(project.building.levels.size()))
      {
        // save the center point of the current level's image coordinates
        const QPoint p_center_window(
          map_view->viewport()->width() / 2,
          map_view->viewport()->height() / 2);
        QPointF p_center_scene = map_view->mapToScene(p_center_window);
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

        double scale = prev_scale *
        project.building.levels[row].drawing_meters_per_pixel /
        project.building.levels[level_idx].drawing_meters_per_pixel;
        if (isnan(scale))
        {
          scale = 1.0;
          p_transformed = QPointF(0.0, 0.0);
        }

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
    [this]() { this->create_scene(); });

  traffic_table = new TrafficTable;
  connect(
    traffic_table,
    &TableList::redraw,
    [this]() { this->create_scene(); });

  connect(
    traffic_table,
    &QTableWidget::cellClicked,
    [=](int row, int /*col*/)
    {
      project.traffic_map_idx = row;
      traffic_table->update(project);
    });

  scenario_table = new ScenarioTable;
  connect(
    scenario_table,
    &QTableWidget::cellClicked,
    [=](int row, int /*col*/)
    {
      project.scenario_row_clicked(row);
      create_scene();
    });

  crowd_sim_table = new CrowdSimEditorTable(project);
  connect(
    crowd_sim_table,
    &QTableWidget::cellClicked,
    [&]()
    {
      crowd_sim_table->update();
      create_scene();
    }
  );


  right_tab_widget = new QTabWidget;
  right_tab_widget->setStyleSheet("QTabBar::tab { color: black; }");
  right_tab_widget->addTab(level_table, "levels");
  right_tab_widget->addTab(layers_table, "layers");
  right_tab_widget->addTab(lift_table, "lifts");
  right_tab_widget->addTab(traffic_table, "traffic");
  right_tab_widget->addTab(scenario_table, "scenarios");
  right_tab_widget->addTab(crowd_sim_table, "crowd_sim");

  property_editor = new QTableWidget;
  property_editor->setStyleSheet(
    "QTableWidget { background-color: #e0e0e0; color: black; gridline-color: #606060; } QLineEdit { background:white; }");
  property_editor->setMinimumSize(600, 200);
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

  QHBoxLayout* param_button_layout = new QHBoxLayout;

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

  QVBoxLayout* right_column_layout = new QVBoxLayout;
  right_column_layout->addWidget(right_tab_widget);

  QLabel* properties_label = new QLabel("Properties");
  properties_label->setStyleSheet("QLabel { color: white; }");
  right_column_layout->addWidget(properties_label);
  right_column_layout->addWidget(property_editor);
  right_column_layout->addLayout(param_button_layout);

  QHBoxLayout* hbox_layout = new QHBoxLayout;
  hbox_layout->addLayout(left_layout, 1);
  hbox_layout->addLayout(right_column_layout);

  QWidget* w = new QWidget();
  w->setMouseTracking(true);
  setMouseTracking(true);
  w->setLayout(hbox_layout);
  w->setStyleSheet("background-color: #404040");
  setCentralWidget(w);

  // PROJECT MENU
  QMenu* project_menu = menuBar()->addMenu("&Project");

  project_menu->addAction(
    "&New...",
    this,
    &Editor::project_new,
    QKeySequence(Qt::CTRL + Qt::Key_N));

  project_menu->addAction(
    "&Open...",
    this,
    &Editor::project_open,
    QKeySequence(Qt::CTRL + Qt::Key_O));

  project_menu->addAction(
    "&Save",
    this,
    &Editor::project_save,
    QKeySequence(Qt::CTRL + Qt::Key_S));

  project_menu->addSeparator();

  project_menu->addAction(
    "E&xit",
    this,
    &QWidget::close,
    QKeySequence(Qt::CTRL + Qt::Key_Q));

  // EDIT MENU
  QMenu* edit_menu = menuBar()->addMenu("&Edit");
  edit_menu->addAction(
    "&Undo",
    this,
    &Editor::edit_undo,
    QKeySequence::Undo);
  edit_menu->addAction(
    "&Redo",
    this,
    &Editor::edit_redo,
    QKeySequence::Redo);
  edit_menu->addSeparator();
  edit_menu->addAction(
    "&Building properties...",
    this,
    &Editor::edit_building_properties);
  edit_menu->addAction(
    "&Project properties...",
    this,
    &Editor::edit_project_properties);
  edit_menu->addSeparator();
  edit_menu->addAction(
    "&Transform...",
    this,
    &Editor::edit_transform);
  edit_menu->addSeparator();
  edit_menu->addAction("&Preferences...", this, &Editor::edit_preferences);

  // MODE MENU
  QMenu* mode_menu = menuBar()->addMenu("&Mode");
  mode_menu->addAction(
    "&Building",
    this,
    [this]() { this->set_mode(MODE_BUILDING, "Building"); },
    QKeySequence(Qt::CTRL + Qt::Key_B));

  mode_menu->addAction(
    "&Traffic lanes",
    this,
    [this]() { this->set_mode(MODE_TRAFFIC, "Traffic"); },
    QKeySequence(Qt::CTRL + Qt::Key_T));

  mode_menu->addAction(
    "&Scenario",
    this,
    [this]() { this->set_mode(MODE_SCENARIO, "Scenario"); },
    QKeySequence(Qt::CTRL + Qt::Key_E));

  mode_menu->addAction(
    "&Crowd Simulation",
    this,
    [this]() { this->set_mode(MODE_CROWD_SIM, "CrowdSim"); },
    QKeySequence(Qt::CTRL + Qt::Key_C));

  // VIEW MENU
  QMenu* view_menu = menuBar()->addMenu("&View");
  view_models_action =
    view_menu->addAction("&Models", this, &Editor::view_models);
  view_models_action->setCheckable(true);
  view_models_action->setChecked(true);
  view_menu->addSeparator();

  zoom_fit_action =
    view_menu->addAction("&Fit to Window", this, &Editor::zoom_fit);
  zoom_fit_action->setEnabled(false);

  // HELP MENU
  QMenu* help_menu = menuBar()->addMenu("&Help");

  help_menu->addAction("&About", this, &Editor::help_about);
  help_menu->addAction("About &Qt", &QApplication::aboutQt);

  // TOOLBAR
  toolbar = new QToolBar();

  mode_combo_box = new QComboBox;
  mode_combo_box->addItem("Building");
  mode_combo_box->addItem("Traffic");
  mode_combo_box->addItem("Scenario");
  mode_combo_box->addItem("Crowd_Sim");
  connect(
    mode_combo_box,
    &QComboBox::currentTextChanged,
    [this](const QString& text)
    {
      if (text == "Building")
        set_mode(MODE_BUILDING, "Building");
      else if (text == "Traffic")
        set_mode(MODE_TRAFFIC, "Traffic");
      else if (text == "Scenario")
        set_mode(MODE_SCENARIO, "Scenario");
      else if (text == "Crowd_Sim")
        set_mode(MODE_CROWD_SIM, "CrowdSim");
    });

  QLabel* mode_label = new QLabel("Edit mode:");
  mode_label->setStyleSheet("QLabel { color: white; }");
  toolbar->addWidget(mode_label);
  toolbar->addWidget(mode_combo_box);

  tool_button_group = new QButtonGroup(this);
  tool_button_group->setExclusive(true);

  create_tool_button(TOOL_SELECT, ":icons/select.svg", "Select (Esc)");
  create_tool_button(TOOL_MOVE, ":icons/move.svg", "Move (M)");
  create_tool_button(TOOL_ROTATE, ":icons/rotate.svg", "Rotate (R)");
  create_tool_button(TOOL_ADD_VERTEX, ":icons/vertex.svg", "Add Vertex (V)");
  create_tool_button(TOOL_ADD_FIDUCIAL, ":icons/fiducial.svg", "Add Fiducial");
  create_tool_button(TOOL_ADD_LANE, "", "Add Lane (L)");
  create_tool_button(TOOL_ADD_WALL, ":icons/wall.svg", "Add Wall (W)");
  create_tool_button(
    TOOL_ADD_MEAS,
    ":icons/measurement.svg",
    "Add Measurement");
  create_tool_button(TOOL_ADD_DOOR, ":icons/door.svg", "Add Door");
  create_tool_button(TOOL_ADD_MODEL, "", "Add Model");
  create_tool_button(TOOL_ADD_FLOOR, ":icons/floor.svg", "Add floor polygon");
  create_tool_button(TOOL_ADD_HOLE, ":icons/hole.svg", "Add hole polygon");
  create_tool_button(TOOL_ADD_ROI, ":icons/roi.svg", "Add region of interest");
  create_tool_button(TOOL_EDIT_POLYGON, "", "Edit Polygon");
  create_tool_button(TOOL_ADD_HUMAN_LANE, "", "Add Human Lane with width");

  connect(
    tool_button_group,
    QOverload<int, bool>::of(&QButtonGroup::buttonToggled),
    this, &Editor::tool_toggled);

  toolbar->addSeparator();

#ifdef HAS_IGNITION_PLUGIN
  sim_reset_action = toolbar->addAction(
    "Reset",
    this,
    &Editor::sim_reset);
  sim_reset_action->setVisible(false);

  sim_play_pause_action = toolbar->addAction(
    "Play",
    this,
    &Editor::sim_play_pause);
  sim_play_pause_action->setVisible(false);

#ifdef HAS_OPENCV
  record_start_stop_action = toolbar->addAction(
    "Record",
    this,
    &Editor::record_start_stop);
  record_start_stop_action->setVisible(false);
#endif  // HAS_OPENCV

#endif  // HAS_IGNITION_PLUGIN

  toolbar->setStyleSheet(
    "QToolBar {background-color: #404040; border: none; spacing: 5px} QToolButton {background-color: #c0c0c0; color: blue; border: 1px solid black;} QToolButton:checked {background-color: #808080; color: red; border: 1px solid black;}");
  addToolBar(Qt::TopToolBarArea, toolbar);

  ///////////////////////////////////////////////////////////
  // SET SIZE
  const int width =
    settings.contains(preferences_keys::window_width) ?
    settings.value(preferences_keys::window_width).toInt() :
    QGuiApplication::primaryScreen()->availableSize().width();

  const int height =
    settings.contains(preferences_keys::window_height) ?
    settings.value(preferences_keys::window_height).toInt() :
    QGuiApplication::primaryScreen()->availableSize().height();

  const int left =
    settings.contains(preferences_keys::window_left) ?
    settings.value(preferences_keys::window_left).toInt() :
    0;

  const int top =
    settings.contains(preferences_keys::window_top) ?
    settings.value(preferences_keys::window_top).toInt() :
    0;

  setGeometry(left, top, width, height);
  map_view->adjustSize();

  // default tool is the "select" tool
  tool_button_group->button(TOOL_SELECT)->click();
  set_mode(MODE_BUILDING, "Building");

  load_model_names();
  level_table->setCurrentCell(level_idx, 0);

#ifdef HAS_IGNITION_PLUGIN
  scene_update_timer = new QTimer;
  connect(
    scene_update_timer,
    &QTimer::timeout,
    this,
    &Editor::scene_update_timer_timeout);
  scene_update_timer->start(1000 / 30);
#endif
}

Editor::~Editor()
{
#if defined(HAS_OPENCV) && defined(HAS_IGNITION_PLUGIN)
  if (video_writer)
  {
    delete video_writer;
    video_writer = nullptr;
  }
#endif
}

#ifdef HAS_IGNITION_PLUGIN
void Editor::scene_update_timer_timeout()
{
  if (project.building.levels.empty())
    return;// let's not crash...

  project.scenario_scene_update(scene, level_idx);

  {
    std::lock_guard<std::mutex> building_guard(
      project.building.building_mutex);

    // project->draw_scenario(scene, level_idx);
    //project.building.levels[level_idx]->name,
    //    building.levels[level_idx]->drawing_meters_per_pixel,

    const BuildingLevel& level = project.building.levels[level_idx];

    const std::string& level_name = level.name;
    const double level_scale = level.drawing_meters_per_pixel;

    // for now, we're not dealing with models changing levels from their
    // starting level. we'll need to do that in the future at some point.
    for (auto& model : project.building.levels[level_idx].models)
    {
      if (!model.is_active)
        continue;

      if (model.state.level_name != level_name)
        continue;

      model.draw(scene, editor_models, level_scale);
    }

    //scenario->draw(scene, level_idx);
  }

#ifdef HAS_OPENCV
  record_frame_to_video();
#endif  // HAS_OPENCV
}
#endif  // HAS_IGNITION_PLUGIN

void Editor::load_model_names()
{
  QSettings settings;
  QString thumbnail_path(
    settings.value(preferences_keys::thumbnail_path).toString());
  if (thumbnail_path.isEmpty())
  {
    std::string assets_dir;
    std::string share_dir;

    try
    {
      share_dir =
        ament_index_cpp::get_package_share_directory("traffic_editor_assets");

      ament_index_cpp::get_resource("traffic_editor_assets",
        "assets",
        assets_dir);
    }
    catch (const ament_index_cpp::PackageNotFoundError& e)
    {
      qWarning("Could not load default thumbnail directory! "
        "traffic_editor_assets package not found in workspace!");
      return;
    }

    // Strip newlines from assets_dir
    assets_dir.erase(std::remove(assets_dir.begin(), assets_dir.end(), '\n'),
      assets_dir.end());

    // Obtain thumbnail path from traffic_editor_assets ament package
    thumbnail_path =
      QDir::cleanPath(
      QDir(QApplication::applicationDirPath()).filePath(
        (share_dir + "/" + assets_dir + "/thumbnails").c_str()
      )
      );
    settings.setValue(preferences_keys::thumbnail_path, thumbnail_path);
  }

  QString model_list_path = QDir(thumbnail_path).filePath("model_list.yaml");

  YAML::Node y;
  std::string filename(model_list_path.toStdString());
  try
  {
    y = YAML::LoadFile(filename);
  }
  catch (const std::exception& e)
  {
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

QToolButton* Editor::create_tool_button(
  const ToolId id,
  const QString& icon_filename,
  const QString& tooltip)
{
  QToolButton* b = new QToolButton(toolbar);
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
  tools[id] = toolbar->addWidget(b);
  tool_button_group->addButton(b, id);
  return b;
}

Editor* Editor::get_instance()
{
  return instance;
}

bool Editor::load_project(const QString& filename)
{
  const std::string filename_std_string = filename.toStdString();

  if (!project.load(filename.toStdString()))
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

  update_tables();

#ifdef HAS_IGNITION_PLUGIN
  if (project.has_sim_plugin())
  {
    printf("project has a sim plugin\n");
    sim_reset_action->setVisible(true);
    sim_play_pause_action->setVisible(true);
#ifdef HAS_OPENCV
    record_start_stop_action->setVisible(true);
#endif
  }
  else
    printf("project does not have a sim plugin\n");
#endif

  QSettings settings;
  settings.setValue(preferences_keys::previous_project_path, filename);

  setWindowModified(false);

  return true;
}

void Editor::restore_previous_viewport()
{
  QSettings settings;

  if (settings.contains(preferences_keys::level_name))
  {
    const std::string level_name =
      settings.value(preferences_keys::level_name).toString().toStdString();
    for (size_t i = 0; i < project.building.levels.size(); i++)
    {
      if (project.building.levels[i].name == level_name)
      {
        level_idx = i;
        create_scene();
        level_table->setCurrentCell(i, 0);
        break;
      }
    }
  }

  double viewport_center_x =
    settings.contains(preferences_keys::viewport_center_x) ?
    settings.value(preferences_keys::viewport_center_x).toDouble() :
    0.0;

  double viewport_center_y =
    settings.contains(preferences_keys::viewport_center_y) ?
    settings.value(preferences_keys::viewport_center_y).toDouble() :
    0.0;

  double viewport_scale =
    settings.contains(preferences_keys::viewport_scale) ?
    settings.value(preferences_keys::viewport_scale).toDouble() :
    1.0;

  // sanity-check the viewport center and scale, since they can be garbage
  // if the last loaded level doesn't have its scale set up.
  if (isnan(viewport_center_x))
    viewport_center_x = 0.0;
  if (isnan(viewport_center_y))
    viewport_center_y = 0.0;
  if (isnan(viewport_scale))
    viewport_scale = 1.0;

  printf("restoring viewport: (%.1f, %.1f, %3f)\n",
    viewport_center_x,
    viewport_center_y,
    viewport_scale);

  QTransform t;
  t.scale(viewport_scale, viewport_scale);
  map_view->setTransform(t);
  map_view->centerOn(QPointF(viewport_center_x, viewport_center_y));
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

void Editor::project_new()
{
  QFileDialog dialog(this, "New Project");
  dialog.setNameFilter("*.project.yaml");
  dialog.setDefaultSuffix(".project.yaml");
  dialog.setAcceptMode(QFileDialog::AcceptMode::AcceptSave);
  dialog.setConfirmOverwrite(true);

  if (dialog.exec() != QDialog::Accepted)
    return;

  QFileInfo file_info(dialog.selectedFiles().first());
  std::string fn = file_info.fileName().toStdString();

  project.clear();
  project.set_filename(file_info.absoluteFilePath().toStdString());
  QString dir_path = file_info.dir().path();
  QDir::setCurrent(dir_path);

  create_scene();
  project_save();
  update_tables();

  QSettings settings;
  settings.setValue(
    preferences_keys::previous_project_path,
    QString::fromStdString(project.get_filename()));
}

void Editor::project_open()
{
  QFileDialog file_dialog(this, "Open Project");
  file_dialog.setFileMode(QFileDialog::ExistingFile);
  file_dialog.setNameFilter("*.project.yaml");

  if (file_dialog.exec() != QDialog::Accepted)
    return;

  QFileInfo file_info(file_dialog.selectedFiles().first());
  if (!file_info.exists())
  {
    QMessageBox::critical(
      this,
      "File does not exist",
      "File does not exist. Cannot open file.");
    return;
  }
  load_project(file_info.filePath());
}

bool Editor::project_save()
{
  project.save();
  setWindowModified(false);
  return true;
}

void Editor::help_about()
{
  QMessageBox::about(this, "About", "Welcome to the Traffic Editor");
}

void Editor::edit_undo()
{
  undo_stack.undo();
  if (
    tool_id == TOOL_ADD_LANE
    || tool_id == TOOL_ADD_WALL)
  {
    clicked_idx = -1;
    prev_clicked_idx = -1;
  }
  create_scene();
  update_property_editor();
  setWindowModified(true);
}

void Editor::edit_redo()
{
  undo_stack.redo();
  create_scene();
  setWindowModified(true);
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

void Editor::edit_project_properties()
{
  ProjectDialog project_dialog(project);
  if (project_dialog.exec() == QDialog::Accepted)
    setWindowModified(true);
}

void Editor::edit_transform()
{
  QDialog dialog;
  Ui::TransformDialog dialog_ui;
  dialog_ui.setupUi(&dialog);
  if (dialog.exec() != QDialog::Accepted)
    return;

  const double rotation =
    dialog_ui.rotate_all_models_line_edit->text().toDouble();
  project.building.rotate_all_models(rotation);
  create_scene();
  setWindowModified(true);
}

void Editor::view_models()
{
  project.rendering_options.show_models = view_models_action->isChecked();
  create_scene();
}

void Editor::zoom_fit()
{
  // todo: implement this for real
  //map_view->set_absolute_scale(1.0);
  map_view->resetMatrix();
}

void Editor::mouse_event(const MouseType t, QMouseEvent* e)
{
  QPointF p;
  if (!is_mouse_event_in_map(e, p))
  {
    e->ignore();
    return;
  }
  if (level_idx >= static_cast<int>(project.building.levels.size()))
  {
    if (t == MOUSE_RELEASE)
    {
      if (project.get_filename().empty())
      {
        QMessageBox::critical(
          this,
          "No project",
          "Please try File->New Project... or File->Open Project...");
      }
      else if (project.building.levels.empty())
      {
        QMessageBox::critical(
          this,
          "No levels defined",
          "No levels defined. Use Level->Add...");
      }
    }
    return;
  }
  // dispatch to individual mouse handler functions to save indenting...
  switch (tool_id)
  {
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
    case TOOL_ADD_HOLE:     mouse_add_hole(t, e, p); break;
    case TOOL_EDIT_POLYGON: mouse_edit_polygon(t, e, p); break;
    case TOOL_ADD_FIDUCIAL: mouse_add_fiducial(t, e, p); break;
    case TOOL_ADD_ROI:      mouse_add_roi(t, e, p); break;
    case TOOL_ADD_HUMAN_LANE: mouse_add_human_lane(t, e, p); break;

    default: break;
  }
  previous_mouse_point = p;
}

void Editor::mousePressEvent(QMouseEvent* e)
{
  mouse_event(MOUSE_PRESS, e);
}

void Editor::mouseReleaseEvent(QMouseEvent* e)
{
  mouse_event(MOUSE_RELEASE, e);
}

void Editor::mouseMoveEvent(QMouseEvent* e)
{
  mouse_event(MOUSE_MOVE, e);
}

bool Editor::is_mouse_event_in_map(QMouseEvent* e, QPointF& p_scene)
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

void Editor::keyPressEvent(QKeyEvent* e)
{
  switch (e->key())
  {
    case Qt::Key_Delete:
      if (project.can_delete_current_selection(level_idx))
      {
        undo_stack.push(new DeleteCommand(&project, level_idx));
        create_scene();
      }
      else
      {
        QMessageBox::critical(
          this,
          "Could not delete item",
          "If deleting a vertex, it must not be in any edges or polygons.");

        project.clear_selection(level_idx);
      }
      break;
    case Qt::Key_S:
    case Qt::Key_Escape:
      tool_button_group->button(TOOL_SELECT)->click();
      project.clear_selection(level_idx);
      clear_current_tool_buffer();
      update_property_editor();
      create_scene();
      break;
    case Qt::Key_V:
      clear_current_tool_buffer();
      tool_button_group->button(TOOL_ADD_VERTEX)->click();
      break;
    case Qt::Key_M:
      clear_current_tool_buffer();
      tool_button_group->button(TOOL_MOVE)->click();
      break;
    case Qt::Key_L:
      clear_current_tool_buffer();
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
      for (auto& edge : project.building.levels[level_idx].edges)
      {
        if (edge.type == Edge::LANE && edge.selected)
        {
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
    case TOOL_ADD_HOLE: return "add hole";
    case TOOL_EDIT_POLYGON: return "&edit polygon";
    case TOOL_ADD_HUMAN_LANE: return "add human lane";
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
        "Click one vertex and then another vertex to add an edge.");
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
      {
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
    }
    else
      tool_button_group->button(TOOL_SELECT)->click();// back to select mode
  }
}

void Editor::update_property_editor()
{
  add_param_button->setEnabled(false);
  delete_param_button->setEnabled(false);

  if (project.building.levels.empty())
    return;

  for (const auto& p : project.building.levels[level_idx].polygons)
  {
    if (p.selected)
    {
      populate_property_editor(p);
      return;
    }
  }

  for (const auto& e : project.building.levels[level_idx].edges)
  {
    if (e.selected)
    {
      populate_property_editor(e);
      return;  // stop after finding the first one
    }
  }

  for (const auto& m : project.building.levels[level_idx].models)
  {
    if (m.selected)
    {
      populate_property_editor(m);
      return;  // stop after finding the first one
    }
  }

  for (const auto& v : project.building.levels[level_idx].vertices)
  {
    if (v.selected)
    {
      populate_property_editor(v);
      return;  // stop after finding the first one
    }
  }

  for (const auto& f : project.building.levels[level_idx].fiducials)
  {
    if (f.selected)
    {
      populate_property_editor(f);
      return;  // stop after finding the first one
    }
  }

  // if we get here, we never found anything :(
  clear_property_editor();
}

QTableWidgetItem* Editor::create_table_item(
  const QString& str,
  bool editable)
{
  QTableWidgetItem* item = new QTableWidgetItem(str);
  if (!editable)
    item->setFlags(Qt::NoItemFlags);
  else
    item->setBackground(QBrush(Qt::white));
  return item;
}

void Editor::property_editor_set_row(
  const int row_idx,
  const QString& label,
  const QString& value,
  const bool editable)
{
  QTableWidgetItem* label_item = new QTableWidgetItem(label);
  label_item->setFlags(Qt::NoItemFlags);

  QTableWidgetItem* value_item = new QTableWidgetItem(value);
  if (!editable)
    value_item->setFlags(Qt::NoItemFlags);
  else
    value_item->setBackground(QBrush(Qt::white));

  property_editor->setItem(row_idx, 0, label_item);
  property_editor->setItem(row_idx, 1, value_item);
}

void Editor::property_editor_set_row(
  const int row_idx,
  const QString& label,
  const int& value,
  const bool editable)
{
  property_editor_set_row(row_idx, label, QString::number(value), editable);
}

void Editor::property_editor_set_row(
  const int row_idx,
  const QString& label,
  const double& value,
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

    AddPropertyCommand* cmd = new AddPropertyCommand(
      &project,
      dialog.get_param_name(),
      Param(dialog.get_param_type()),
      level_idx
    );

    undo_stack.push(cmd);
    auto updated_id = cmd->get_vertex_updated();
    populate_property_editor(
      project.building.levels[level_idx].vertices[updated_id]);
    setWindowModified(true);
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
    return;// let's not crash...
  const Level& level = project.building.levels[level_idx];
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
  QPushButton* add_button = new QPushButton("Add...", this);
  layers_table->setCellWidget(last_row_idx, 1, add_button);
  connect(
    add_button, &QAbstractButton::clicked,
    [=]() { this->layer_add_button_clicked(); });

  layers_table->blockSignals(false);  // re-enable callbacks
}

void Editor::layers_table_set_row(
  const int row_idx,
  const QString& label,
  const bool checked)
{
  QCheckBox* checkbox = new QCheckBox(label);
  checkbox->setChecked(checked);
  layers_table->setCellWidget(row_idx, 0, checkbox);

  QPushButton* button = new QPushButton("Edit...", this);
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

void Editor::layer_edit_button_clicked(const std::string& label)
{
  printf("clicked: [%s]\n", label.c_str());
  if (project.building.levels.empty())
    return;
  // find the index of this layer in the current level
  Level& level = project.building.levels[level_idx];
  for (size_t i = 0; i < level.layers.size(); i++)
  {
    Layer& layer = level.layers[i];
    if (label != layer.name)
      continue;
    LayerDialog* dialog = new LayerDialog(this, layer, true);
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
    return;// let's not crash (yet)
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
  const Vertex& sv = level.vertices[edge.start_idx];
  const Vertex& ev = level.vertices[edge.end_idx];

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
  for (const auto& param : edge.params)
  {
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

  property_editor_set_row(0, "x (pixels)", vertex.x, 3, true);
  property_editor_set_row(1, "y (pixels)", vertex.y, 3, true);
  property_editor_set_row(2, "x (m)", vertex.x * scale);
  property_editor_set_row(3, "y (m)", -1.0 * vertex.y * scale);
  property_editor_set_row(
    4,
    "name",
    QString::fromStdString(vertex.name),
    true);

  int row = 5;
  for (const auto& param : vertex.params)
  {
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

  property_editor->setRowCount(4);

  property_editor_set_row(
    0,
    "name",
    QString::fromStdString(model.instance_name),
    true);

  property_editor_set_row(
    1,
    "model_name",
    QString::fromStdString(model.model_name));

  property_editor_set_row(
    2,
    "elevation",
    model.state.z,
    3,
    true);

  property_editor_set_row(
    3,
    "static",
    model.is_static ? QString("true") : QString("false"),
    true);

  property_editor->blockSignals(false);  // re-enable callbacks
}

void Editor::populate_property_editor(const Polygon& polygon)
{
  printf("populate_property_editor(polygon)\n");
  property_editor->blockSignals(true);  // otherwise we get tons of callbacks

  property_editor->setRowCount(polygon.params.size());

  int row = 0;
  for (const auto& param : polygon.params)
  {
    property_editor_set_row(
      row,
      QString::fromStdString(param.first),
      param.second.to_qstring(),
      true);
    row++;
  }

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
    else if (name == "x (pixels)")
      v.x = stof(value);
    else if (name == "y (pixels)")
      v.y = stof(value);
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

  for (auto& p : project.building.levels[level_idx].polygons)
  {
    if (!p.selected)
      continue;
    p.set_param(name, value);
    setWindowModified(true);
    return;  // stop after finding the first one
  }

  for (auto& m : project.building.levels[level_idx].models)
  {
    if (!m.selected)
      continue;
    m.set_param(name, value);
    setWindowModified(true);
    return; // stop after finding the first one
  }
}

bool Editor::create_scene()
{
  scene->clear();  // destroys the mouse_motion_* items if they are there
#ifdef HAS_IGNITION_PLUGIN
  project.clear_scene();  // forget all pointers to the graphics items
#endif
  mouse_motion_line = nullptr;
  mouse_motion_model = nullptr;
  mouse_motion_ellipse = nullptr;
  mouse_motion_polygon = nullptr;

  project.draw(scene, level_idx, editor_models);

  return true;
}

void Editor::draw_mouse_motion_line_item(
  const double mouse_x,
  const double mouse_y)
{
  double pen_width = 1;
  QColor color;
  switch (tool_id)
  {
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
  const auto& start =
    project.building.levels[level_idx].vertices[clicked_idx];
  if (!mouse_motion_line)
    mouse_motion_line = scene->addLine(start.x, start.y, mouse_x, mouse_y, pen);
  else
    mouse_motion_line->setLine(start.x, start.y, mouse_x, mouse_y);
}

void Editor::remove_mouse_motion_item()
{
  if (mouse_motion_line)
  {
    scene->removeItem(mouse_motion_line);
    delete mouse_motion_line;
    mouse_motion_line = nullptr;
  }
  if (mouse_motion_model)
  {
    scene->removeItem(mouse_motion_model);
    delete mouse_motion_model;
    mouse_motion_model = nullptr;
  }
  if (mouse_motion_ellipse)
  {
    scene->removeItem(mouse_motion_ellipse);
    delete mouse_motion_ellipse;
    mouse_motion_ellipse = nullptr;
  }
  if (mouse_motion_polygon)
  {
    scene->removeItem(mouse_motion_polygon);
    delete mouse_motion_polygon;
    mouse_motion_polygon = nullptr;
  }
  mouse_motion_editor_model = nullptr;

  mouse_vertex_idx = -1;
  mouse_fiducial_idx = -1;
}

///////////////////////////////////////////////////////////////////////
// MOUSE HANDLERS
///////////////////////////////////////////////////////////////////////

void Editor::mouse_select(
  const MouseType type, QMouseEvent* e, const QPointF& p)
{
  if (type != MOUSE_PRESS)
    return;
  const QPoint p_global = mapToGlobal(e->pos());
  const QPoint p_map = map_view->mapFromGlobal(p_global);
  QGraphicsItem* item = map_view->itemAt(p_map);

  project.mouse_select_press(mode, level_idx, p.x(), p.y(), item);

  // todo: figure out something smarter than this abomination
  selected_polygon = project.get_selected_polygon(mode, level_idx);

  // todo: be smarter and go find the actual GraphicsItem to avoid
  // a full repaint here?
  create_scene();
  update_property_editor();
}

void Editor::mouse_add_vertex(
  const MouseType t, QMouseEvent*, const QPointF& p)
{
  if (t == MOUSE_PRESS)
  {
    if (mode == MODE_SCENARIO)
    {
      if (project.scenario_idx < 0)
      {
        QMessageBox::warning(
          this,
          "Add Vertex",
          "No scenario currently defined.");
        return;
      }
    }

    AddVertexCommand* command = new AddVertexCommand(&project, mode, level_idx,
        p.x(), p.y());

    undo_stack.push(command);

    setWindowModified(true);
    create_scene();
  }
}

void Editor::mouse_add_fiducial(
  const MouseType t, QMouseEvent*, const QPointF& p)
{
  if (t == MOUSE_PRESS)
  {
    AddFiducialCommand* command = new AddFiducialCommand(
      &project,
      level_idx,
      p.x(),
      p.y());
    undo_stack.push(command);
    setWindowModified(true);
    create_scene();
  }
}

void Editor::mouse_move(
  const MouseType t, QMouseEvent* e, const QPointF& p)
{
  if (t == MOUSE_PRESS)
  {
    Building::NearestItem ni =
      project.building.nearest_items(level_idx, p.x(), p.y());

    // todo: use QGraphics stuff to see if we clicked a model pixmap...
    const double model_dist_thresh = 0.5 /
      project.building.levels[level_idx].drawing_meters_per_pixel;

    if (ni.model_idx >= 0 && ni.model_dist < model_dist_thresh)
    {
      // Now we need to find the pixmap item for this model.
      const Model& model =
        project.building.levels[level_idx].models[ni.model_idx];
      mouse_motion_model = get_closest_pixmap_item(
        QPointF(model.state.x, model.state.y));
      mouse_model_idx = ni.model_idx;
      latest_move_model = new MoveModelCommand(&project, level_idx,
          mouse_model_idx);
    }
    else if (ni.vertex_idx >= 0 && ni.vertex_dist < 10.0)
    {
      mouse_vertex_idx = ni.vertex_idx;

      latest_move_vertex = new MoveVertexCommand(&project, level_idx,
          mouse_vertex_idx);
      // todo: save the QGrahpicsEllipse or group, to avoid full repaints?
    }
    else if (ni.fiducial_idx >= 0 && ni.fiducial_dist < 10.0)
    {
      mouse_fiducial_idx = ni.fiducial_idx;
      latest_move_fiducial = new MoveFiducialCommand(&project, level_idx,
          mouse_fiducial_idx);
      // todo: save the QGrahpicsEllipse or group, to avoid full repaints?
    }
  }
  else if (t == MOUSE_RELEASE)
  {
    if (mouse_vertex_idx >= 0) //Add mouse move vertex.
    {
      if (latest_move_vertex->has_moved)
      {
        undo_stack.push(latest_move_vertex);
      }
      else
      {
        delete latest_move_vertex;
        latest_move_vertex = NULL;
      }
    }

    if (mouse_model_idx >= 0) //Add mouse move model
    {
      if (latest_move_model->has_moved)
      {
        undo_stack.push(latest_move_model);
      }
      else
      {
        delete latest_move_model;
        latest_move_model = NULL;
      }
    }

    if (mouse_fiducial_idx >= 0) //Add mouse move fiducial
    {
      if (latest_move_fiducial->has_moved)
      {
        undo_stack.push(latest_move_fiducial);
      }
      else
      {
        delete latest_move_fiducial;
        latest_move_fiducial = NULL;
      }
    }
    mouse_vertex_idx = -1;
    mouse_fiducial_idx = -1;
    create_scene();  // this will free mouse_motion_model
    setWindowModified(true);
  }
  else if (t == MOUSE_MOVE)
  {
    if (!(e->buttons() & Qt::LeftButton))
      return;// we only care about mouse-dragging, not just motion
    printf("mouse move, vertex_idx = %d, fiducial_idx = %d\n",
      mouse_vertex_idx,
      mouse_fiducial_idx);
    if (mouse_motion_model != nullptr)
    {
      // we're dragging a model
      // update both the nav_model data and the pixmap in the scene
      Model& model =
        project.building.levels[level_idx].models[mouse_model_idx];
      model.state.x = p.x();
      model.state.y = p.y();
      mouse_motion_model->setPos(p);
      latest_move_model->set_final_destination(p.x(), p.y());
    }
    else if (mouse_vertex_idx >= 0)
    {
      // we're dragging a vertex
      Vertex& pt =
        project.building.levels[level_idx].vertices[mouse_vertex_idx];
      pt.x = p.x();
      pt.y = p.y();
      latest_move_vertex->set_final_destination(p.x(), p.y());
      create_scene();
    }
    else if (mouse_fiducial_idx >= 0)
    {
      Fiducial& f =
        project.building.levels[level_idx].fiducials[mouse_fiducial_idx];
      f.x = p.x();
      f.y = p.y();
      latest_move_fiducial->set_final_destination(p.x(), p.y());
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
  QMouseEvent* e,
  const QPointF& p,
  const Edge::Type& edge_type)
{
  QPointF p_aligned(p);
  if (clicked_idx >= 0 && e->modifiers() & Qt::ShiftModifier)
  {
    const auto& start =
      project.building.levels[level_idx].vertices[clicked_idx];
    align_point(QPointF(start.x, start.y), p_aligned);
  }

  if (t == MOUSE_PRESS)
  {
    if (e->buttons() & Qt::RightButton)
    {
      // right button means "exit edge drawing mode please"
      clicked_idx = -1;
      prev_clicked_idx = -1;
      if (latest_add_edge != NULL)
      {
        //Need to check if new vertex was added.
        delete latest_add_edge;
        latest_add_edge = NULL;
      }
      remove_mouse_motion_item();
      return;
    }

    if (prev_clicked_idx < 0)
    {
      latest_add_edge = new AddEdgeCommand(&project, level_idx);
      clicked_idx = latest_add_edge->set_first_point(p_aligned.x(),
          p_aligned.y());
      latest_add_edge->set_edge_type(edge_type);
      prev_clicked_idx = clicked_idx;
      create_scene();
      setWindowModified(true);
      return; // no previous vertex click happened; nothing else to do
    }

    clicked_idx =
      latest_add_edge->set_second_point(p_aligned.x(), p_aligned.y());

    if (clicked_idx == prev_clicked_idx)  // don't create self edge loops
    {
      remove_mouse_motion_item();
      return;
    }
    undo_stack.push(latest_add_edge);

    if (edge_type == Edge::DOOR || edge_type == Edge::MEAS)
    {
      clicked_idx = -1;  // doors and measurements don't usually chain
      latest_add_edge = NULL;
      remove_mouse_motion_item();
    }
    else
    {
      latest_add_edge = new AddEdgeCommand(&project, level_idx);
      latest_add_edge->set_first_point(p_aligned.x(), p_aligned.y());
      latest_add_edge->set_edge_type(edge_type);
    }
    prev_clicked_idx = clicked_idx;
    create_scene();
    setWindowModified(true);
  }
  else if (t == MOUSE_MOVE)
  {
    if (clicked_idx < 0)
      return;

    draw_mouse_motion_line_item(p_aligned.x(), p_aligned.y());
  }
}

void Editor::mouse_add_lane(
  const MouseType t, QMouseEvent* e, const QPointF& p)
{
  mouse_add_edge(t, e, p, Edge::LANE);
}

void Editor::mouse_add_wall(
  const MouseType t, QMouseEvent* e, const QPointF& p)
{
  mouse_add_edge(t, e, p, Edge::WALL);
}

void Editor::mouse_add_meas(
  const MouseType t, QMouseEvent* e, const QPointF& p)
{
  mouse_add_edge(t, e, p, Edge::MEAS);
}

void Editor::mouse_add_door(
  const MouseType t, QMouseEvent* e, const QPointF& p)
{
  mouse_add_edge(t, e, p, Edge::DOOR);
}

void Editor::mouse_add_human_lane(
  const MouseType t, QMouseEvent* e, const QPointF& p)
{
  mouse_add_edge(t, e, p, Edge::HUMAN_LANE);
}

void Editor::mouse_add_model(
  const MouseType t, QMouseEvent*, const QPointF& p)
{
  if (t == MOUSE_PRESS)
  {
    if (mouse_motion_editor_model == nullptr)
      return;

    AddModelCommand* cmd = new AddModelCommand(
      &project,
      level_idx,
      p.x(),
      p.y(),
      mouse_motion_editor_model->name
    );
    undo_stack.push(cmd);
    setWindowModified(true);
    create_scene();
  }
  else if (t == MOUSE_MOVE)
  {
    if (mouse_motion_editor_model == nullptr)
      return;// nothing currently selected. nothing to do.
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

double Editor::discretize_angle(const double& angle)
{
  const double discretization = 45.0 * M_PI / 180.0;
  return discretization * round(angle / discretization);
}

void Editor::align_point(const QPointF& start, QPointF& end)
{
  if (qAbs(start.x() - end.x()) < qAbs(start.y() - end.y()))
    end.setX(start.x());
  else
    end.setY(start.y());
}

void Editor::mouse_rotate(
  const MouseType t, QMouseEvent* mouse_event, const QPointF& p)
{
  if (t == MOUSE_PRESS)
  {
    clicked_idx = project.building.nearest_item_index_if_within_distance(
      level_idx,
      p.x(),
      p.y(),
      50.0,
      Building::MODEL);
    if (clicked_idx < 0)
      return;// nothing to do. click wasn't on a model.

    latest_rotate_model = new RotateModelCommand(&project, level_idx,
        clicked_idx);
    const Model& model =
      project.building.levels[level_idx].models[clicked_idx];
    mouse_motion_model = get_closest_pixmap_item(
      QPointF(model.state.x, model.state.y));
    QPen pen(Qt::red);
    pen.setWidth(4);
    const double r = static_cast<double>(ROTATION_INDICATOR_RADIUS);
    mouse_motion_ellipse = scene->addEllipse(
      model.state.x - r,  // ellipse upper-left column
      model.state.y - r,  // ellipse upper-left row
      2 * r,  // ellipse width
      2 * r,  // ellipse height
      pen);
    mouse_motion_line = scene->addLine(
      model.state.x,
      model.state.y,
      model.state.x + r * cos(model.state.yaw),
      model.state.y - r * sin(model.state.yaw),
      pen);
  }
  else if (t == MOUSE_RELEASE)
  {
    //remove_mouse_motion_item();
    if (clicked_idx < 0)
      return;
    const Model& model =
      project.building.levels[level_idx].models[clicked_idx];
    const double dx = p.x() - model.state.x;
    const double dy = -(p.y() - model.state.y);  // vertical axis is flipped
    double mouse_yaw = atan2(dy, dx);
    if (mouse_event->modifiers() & Qt::ShiftModifier)
      mouse_yaw = discretize_angle(mouse_yaw);
    latest_rotate_model->set_final_destination(mouse_yaw);
    undo_stack.push(latest_rotate_model);
    clicked_idx = -1;  // we're done rotating it now
    setWindowModified(true);
    // now re-render the whole scene (could optimize in the future...)
    create_scene();
  }
  else if (t == MOUSE_MOVE)
  {
    if (clicked_idx < 0)
      return;// nothing currently selected. nothing to do.

    // re-orient the mouse_motion_model item and heading indicator as needed
    const Model& model =
      project.building.levels[level_idx].models[clicked_idx];
    const double dx = p.x() - model.state.x;
    const double dy = -(p.y() - model.state.y);  // vertical axis is flipped
    double mouse_yaw = atan2(dy, dx);
    if (mouse_event->modifiers() & Qt::ShiftModifier)
      mouse_yaw = discretize_angle(mouse_yaw);
    const double r = static_cast<double>(ROTATION_INDICATOR_RADIUS);
    mouse_motion_line->setLine(
      model.state.x,
      model.state.y,
      model.state.x + r * cos(mouse_yaw),
      model.state.y - r * sin(mouse_yaw));

    if (mouse_motion_model)
      mouse_motion_model->setRotation(
        (-mouse_yaw + M_PI / 2.0) * 180.0 / M_PI);
  }
}

QGraphicsPixmapItem* Editor::get_closest_pixmap_item(const QPointF& p)
{
  // todo: use fancier calls if the scene graph gets so big that a linear
  // search becomes intolerably slow
  const QList<QGraphicsItem*> items = scene->items();
  QGraphicsPixmapItem* pixmap_item = nullptr;
  double min_dist = 1.0e9;
  for (const auto item : items)
  {
    if (item->type() != QGraphicsPixmapItem::Type)
      continue;// ignore anything other than the pixmaps (models)
    const double model_click_distance = QLineF(p, item->pos()).length();
    const double width = item->boundingRect().width();
    const double height = item->boundingRect().height();
    printf("model_click_distance = %.2f bounds = (%.1f, %.1f)\n",
      model_click_distance, width, height);
    if (model_click_distance < min_dist)
    {
      min_dist = model_click_distance;
      pixmap_item = qgraphicsitem_cast<QGraphicsPixmapItem*>(item);
    }
  }
  return pixmap_item;
}

void Editor::mouse_add_polygon(
  const MouseType t,
  QMouseEvent* e,
  const QPointF& p,
  const Polygon::Type& polygon_type)
{
  if (t == MOUSE_PRESS)
  {
    if (e->buttons() & Qt::LeftButton)
    {
      const Project::NearestItem ni =
        project.nearest_items(mode, level_idx, p.x(), p.y());
      clicked_idx = ni.vertex_dist < 10.0 ? ni.vertex_idx : -1;
      if (clicked_idx < 0)
        return;// nothing to do. click wasn't on a vertex.

      Vertex* v = nullptr;
      if (mode == MODE_BUILDING)
        v = &project.building.levels[level_idx].vertices[clicked_idx];
      else if (mode == MODE_SCENARIO)
        v = &project.scenario_level(level_idx)->vertices[clicked_idx];

      v->selected = true;  // todo: colorize it?

      if (mouse_motion_polygon == nullptr)
      {
        QVector<QPointF> polygon_vertices;
        polygon_vertices.append(QPointF(v->x, v->y));
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
          clicked_idx) == mouse_motion_polygon_vertices.end())
        mouse_motion_polygon_vertices.push_back(clicked_idx);
    }
    else if (e->buttons() & Qt::RightButton)
    {
      if (mouse_motion_polygon == nullptr)
        return;
      if (mouse_motion_polygon_vertices.size() >= 3)
      {
        Polygon polygon;
        polygon.type = polygon_type;
        polygon.create_required_parameters();
        for (const auto& i : mouse_motion_polygon_vertices)
        {
          polygon.vertices.push_back(i);
        }

        AddPolygonCommand* command = new AddPolygonCommand(
          &project,
          mode,
          polygon,
          level_idx);

        undo_stack.push(command);
      }
      scene->removeItem(mouse_motion_polygon);
      delete mouse_motion_polygon;
      mouse_motion_polygon = nullptr;

      setWindowModified(true);
      project.clear_selection(level_idx);
      create_scene();
    }
  }
  else if (t == MOUSE_MOVE)
  {
    if (mouse_motion_polygon == nullptr)
      return;

    // first, remove the previous polygon
    scene->removeItem(mouse_motion_polygon);
    delete mouse_motion_polygon;

    // now, make the updated polygon
    QVector<QPointF> polygon_vertices;
    for (const auto& vertex_idx: mouse_motion_polygon_vertices)
    {
      const Vertex* v = nullptr;
      if (mode == MODE_BUILDING)
        v = &project.building.levels[level_idx].vertices[vertex_idx];
      else if (mode == MODE_SCENARIO)
        v = &project.scenario_level(level_idx)->vertices[vertex_idx];

      polygon_vertices.append(QPointF(v->x, v->y));
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
  const MouseType t, QMouseEvent* e, const QPointF& p)
{
  mouse_add_polygon(t, e, p, Polygon::FLOOR);
}

void Editor::mouse_add_hole(
  const MouseType t, QMouseEvent* e, const QPointF& p)
{
  mouse_add_polygon(t, e, p, Polygon::HOLE);
}

void Editor::mouse_add_roi(
  const MouseType t, QMouseEvent* e, const QPointF& p)
{
  mouse_add_polygon(t, e, p, Polygon::ROI);
}

void Editor::mouse_edit_polygon(
  const MouseType t, QMouseEvent* e, const QPointF& p)
{
  if (selected_polygon == nullptr)
    return;// no polygon is selected, nothing to do

  if (t == MOUSE_PRESS)
  {
    if (e->buttons() & Qt::RightButton)
    {
      const Project::NearestItem ni = project.nearest_items(
        mode, level_idx, p.x(), p.y());
      if (ni.vertex_dist > 10.0)
      {
        printf("right-click wasn't near a vertex: %.1f\n", ni.vertex_dist);
        return;  // click wasn't near a vertex
      }
      else
      {
        printf("removing vertex %d\n", ni.vertex_idx);
      }
      PolygonRemoveVertCommand* command = new PolygonRemoveVertCommand(
        selected_polygon, ni.vertex_idx);
      undo_stack.push(command);
      setWindowModified(true);
      create_scene();
    }
    else if (e->buttons() & Qt::LeftButton)
    {
      mouse_edge_drag_polygon =
        project.polygon_edge_drag_press(
        mode,
        level_idx,
        selected_polygon,
        p.x(),
        p.y());
      if (mouse_edge_drag_polygon.movable_vertex < 0)
        return;

      if (mouse_motion_polygon != nullptr)
      {
        qWarning("edit_polygon_release() without null mouse_motion_polygon!");
        return;
      }

      mouse_motion_polygon = scene->addPolygon(
        mouse_edge_drag_polygon.polygon,
        QPen(Qt::black),
        QBrush(QColor::fromRgbF(1.0, 1.0, 0.5, 0.5)));
    }
  }
  else if (t == MOUSE_RELEASE)
  {
    // todo by drag mode (left/right button?)
    if (mouse_motion_polygon == nullptr)
    {
      qInfo("woah! edit_polygon_release() with null mouse_motion_polygon!");
      return;
    }
    printf("replacing vertices of polygon...\n");
    scene->removeItem(mouse_motion_polygon);
    delete mouse_motion_polygon;
    mouse_motion_polygon = nullptr;

    const Project::NearestItem ni = project.nearest_items(
      mode, level_idx, p.x(), p.y());

    if (ni.vertex_dist > 10.0)
      return;// nothing to do; didn't release near a vertex

    const int release_vertex_idx = ni.vertex_idx;

    if (std::find(
        selected_polygon->vertices.begin(),
        selected_polygon->vertices.end(),
        release_vertex_idx) != selected_polygon->vertices.end())
      return;// Release vertex is already in the polygon. Don't do anything.

    PolygonAddVertCommand* command = new PolygonAddVertCommand(
      selected_polygon,
      mouse_edge_drag_polygon.movable_vertex,
      release_vertex_idx);

    undo_stack.push(command);

    setWindowModified(true);
    create_scene();
  }
  else if (t == MOUSE_MOVE)
  {
    if (e->buttons() & Qt::LeftButton)
    {
      if (mouse_motion_polygon == nullptr)
      {
        qInfo("woah! edit_polygon_release() with null mouse_motion_polygon!");
        return;
      }
      QPolygonF polygon = mouse_motion_polygon->polygon();
      polygon[mouse_edge_drag_polygon.movable_vertex] = QPointF(p.x(), p.y());
      mouse_motion_polygon->setPolygon(polygon);
    }
  }
}

void Editor::number_key_pressed(const int n)
{
  bool found_edge = false;
  for (auto& edge : project.building.levels[level_idx].edges)
  {
    if (edge.selected && edge.type == Edge::LANE)
    {
      edge.set_graph_idx(n);
      found_edge = true;
    }
  }
  if (found_edge)
  {
    create_scene();
    update_property_editor();
  }

  project.traffic_map_idx = n;
  traffic_table->update(project);
}

bool Editor::maybe_save()
{
  if (!isWindowModified())
    return true;// no need to ask to save the document
  const QMessageBox::StandardButton button_clicked =
    QMessageBox::warning(
    this,
    "Project not saved!",
    "Do you want to save your changes?",
    QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
  switch (button_clicked)
  {
    case QMessageBox::Save:
      return project_save();
    case QMessageBox::Cancel:
      return false;
    default:
      break;
  }
  return true;
}

void Editor::showEvent(QShowEvent* event)
{
  QMainWindow::showEvent(event);
#ifdef HAS_IGNITION_PLUGIN
  sim_thread.start();
#endif
}

void Editor::closeEvent(QCloseEvent* event)
{
#ifdef HAS_IGNITION_PLUGIN
  printf("waiting on sim_thread...\n");
  sim_thread.requestInterruption();
  sim_thread.quit();
  sim_thread.wait();
#endif

  // save window geometry
  QSettings settings;
  settings.setValue(preferences_keys::window_left, geometry().x());
  settings.setValue(preferences_keys::window_top, geometry().y());
  settings.setValue(preferences_keys::window_width, geometry().width());
  settings.setValue(preferences_keys::window_height, geometry().height());

  // save viewport center and scale
  const QPoint p_center_window(
    map_view->viewport()->width() / 2,
    map_view->viewport()->height() / 2);
  const QPointF p_center_scene = map_view->mapToScene(p_center_window);

  printf("closeEvent:  (%d, %d) -> (%.1f, %.1f)\n",
    p_center_window.x(),
    p_center_window.y(),
    p_center_scene.x(),
    p_center_scene.y());

  const double scale = map_view->transform().m11();
  settings.setValue(preferences_keys::viewport_center_x, p_center_scene.x());
  settings.setValue(preferences_keys::viewport_center_y, p_center_scene.y());
  settings.setValue(preferences_keys::viewport_scale, scale);

  if (!project.building.levels.empty())
    settings.setValue(
      preferences_keys::level_name,
      QString::fromStdString(project.building.levels[level_idx].name));

  if (maybe_save())
    event->accept();
  else
    event->ignore();
}

void Editor::set_tool_visibility(const ToolId id, const bool visible)
{
  QAction* a = tools[id];
  if (a)
    a->setVisible(visible);
  else
    printf("unable to find tool action %d\n", static_cast<int>(id));
}

void Editor::set_mode(const EditorModeId _mode, const QString& mode_string)
{
  if (mode_combo_box->currentText() != mode_string)
  {
    mode_combo_box->blockSignals(true);
    mode_combo_box->setCurrentText(mode_string);
    mode_combo_box->blockSignals(false);
  }

  mode = _mode;

  // building tools
  set_tool_visibility(TOOL_ADD_WALL, mode == MODE_BUILDING);
  set_tool_visibility(TOOL_ADD_MEAS, mode == MODE_BUILDING);
  set_tool_visibility(TOOL_ADD_DOOR, mode == MODE_BUILDING);
  set_tool_visibility(TOOL_ADD_MODEL, mode == MODE_BUILDING);
  set_tool_visibility(TOOL_ADD_FLOOR, mode == MODE_BUILDING);
  set_tool_visibility(TOOL_ADD_HOLE, mode == MODE_BUILDING);
  set_tool_visibility(TOOL_ADD_FIDUCIAL, mode == MODE_BUILDING);

  // traffic tools
  set_tool_visibility(TOOL_ADD_LANE, mode == MODE_TRAFFIC);

  // scenario tools
  set_tool_visibility(TOOL_ADD_ROI, mode == MODE_SCENARIO);

  // crowd_sim tools
  set_tool_visibility(TOOL_ADD_HUMAN_LANE, mode == MODE_CROWD_SIM);

  // "multi-purpose" tools
  set_tool_visibility(TOOL_EDIT_POLYGON,
    mode != MODE_TRAFFIC && mode != MODE_CROWD_SIM);
}

void Editor::update_tables()
{
  populate_layers_table();
  level_table->update(project.building);
  lift_table->update(project.building);
  scenario_table->update(project);
  traffic_table->update(project);
  crowd_sim_table->update();
}

void Editor::clear_current_tool_buffer()
{
  if (
    tool_id == TOOL_ADD_WALL
    || tool_id == TOOL_ADD_LANE
    || tool_id == TOOL_ADD_MEAS
    || tool_id == TOOL_ADD_HUMAN_LANE
    || tool_id == TOOL_ADD_DOOR)
  {
    prev_clicked_idx = -1;
    clicked_idx = -1;
    delete latest_add_edge;
    latest_add_edge = NULL;
  }
}

#ifdef HAS_IGNITION_PLUGIN
void Editor::sim_reset()
{
  printf("TODO: sim_reset()\n");
  // todo: signal to the sim thread to reset the project
}

void Editor::sim_play_pause()
{
  printf("sim_play_pause()\n");
  project.sim_is_paused = !project.sim_is_paused;
}

void Editor::sim_tick()
{
  // called from sim thread

  if (!project.sim_is_paused)
    project.sim_tick();
}

#ifdef HAS_OPENCV
void Editor::record_start_stop()
{
  is_recording = !is_recording;
}

void Editor::record_frame_to_video()
{
  if (!is_recording)
    return;

  QPixmap pixmap = map_view->viewport()->grab();
  const int w = pixmap.size().width();
  const int h = pixmap.size().height();
  QImage image(pixmap.toImage());
  // int format = static_cast<int>(image.format());
  cv::Mat mat(
    h,
    w,
    CV_8UC4,
    const_cast<uchar*>(image.bits()),
    static_cast<size_t>(image.bytesPerLine()));
  cv::Mat mat_rgb_swap;
  cv::cvtColor(mat, mat_rgb_swap, cv::COLOR_RGBA2BGRA);

  if (video_writer == nullptr)
  {
    printf("initializing video writer...\n");
    video_writer =
      new cv::VideoWriter(
      "test.avi",
      cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
      30,
      cv::Size(w, h));
  }

  video_writer->write(mat_rgb_swap);
}
#endif  // HAS_OPENCV

#endif  // HAS_IGNITION_PLUGIN
