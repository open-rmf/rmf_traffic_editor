cmake_minimum_required(VERSION 3.10.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

project(rmf_traffic_editor)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Qt5 COMPONENTS Widgets Concurrent Test Network REQUIRED)
find_package(yaml-cpp REQUIRED)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
# set(CMAKE_VERBOSE_MAKEFILE TRUE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wshadow -Wextra")

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
  set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

include_directories(.)
include_directories(gui)
include_directories(include)

set(gui_sources
  gui/actions/add_constraint.cpp
  gui/actions/add_edge.cpp
  gui/actions/add_feature.cpp
  gui/actions/add_fiducial.cpp
  gui/actions/add_model.cpp
  gui/actions/add_polygon.cpp
  gui/actions/add_property.cpp
  gui/actions/add_vertex.cpp
  gui/actions/delete.cpp
  gui/actions/move_feature.cpp
  gui/actions/move_fiducial.cpp
  gui/actions/move_model.cpp
  gui/actions/move_vertex.cpp
  gui/actions/polygon_remove_vertices.cpp
  gui/actions/polygon_add_vertex.cpp
  gui/actions/rotate_model.cpp
  gui/add_param_dialog.cpp
  gui/building.cpp
  gui/building_dialog.cpp
  gui/constraint.cpp
  gui/coordinate_system.cpp
  gui/feature.cpp
  gui/edge.cpp
  gui/editor.cpp
  gui/editor_model.cpp
  gui/fiducial.cpp
  gui/graph.cpp
  gui/layer.cpp
  gui/layer_dialog.cpp
  gui/layer_table.cpp
  gui/level.cpp
  gui/level_dialog.cpp
  gui/level_table.cpp
  gui/lift.cpp
  gui/lift_dialog.cpp
  gui/lift_door.cpp
  gui/lift_table.cpp
  gui/map_tile_cache.cpp
  gui/map_view.cpp
  gui/model.cpp
  gui/model_dialog.cpp
  gui/param.cpp
  gui/polygon.cpp
  gui/preferences_dialog.cpp
  gui/preferences_keys.cpp
  gui/rendering_options.cpp
  gui/table_list.cpp
  gui/traffic_table.cpp
  gui/traffic_map.cpp
  gui/transform.cpp
  gui/vertex.cpp
  gui/yaml_utils.cpp

  #crowd_sim related
  gui/multi_select_combo_box.cpp
  gui/crowd_sim/agent_group.cpp
  gui/crowd_sim/agent_group_table.cpp
  gui/crowd_sim/agent_profile.cpp
  gui/crowd_sim/agent_profile_table.cpp
  gui/crowd_sim/condition.cpp
  gui/crowd_sim/condition_dialog.cpp
  gui/crowd_sim/crowd_sim_dialog.cpp
  gui/crowd_sim/crowd_sim_editor_table.cpp
  gui/crowd_sim/crowd_sim_impl.cpp
  gui/crowd_sim/crowd_sim_table_base.cpp
  gui/crowd_sim/goal_set.cpp
  gui/crowd_sim/goal_set_table.cpp
  gui/crowd_sim/model_type.cpp
  gui/crowd_sim/model_type_table.cpp
  gui/crowd_sim/state.cpp
  gui/crowd_sim/state_table.cpp
  gui/crowd_sim/to_state_dialog.cpp
  gui/crowd_sim/to_state_table.cpp
  gui/crowd_sim/transition.cpp
  gui/crowd_sim/transition_table.cpp
)

add_library(
  gui_lib STATIC
  ${gui_sources})

target_link_libraries(
  gui_lib
  ament_index_cpp::ament_index_cpp
  ceres
  Eigen3::Eigen
  Qt5::Widgets
  Qt5::Concurrent
  Qt5::Network
  proj
  yaml-cpp
  ${ament_index_cpp_LIBRARIES}
)

add_executable(
  traffic-editor
  resources/resource.qrc
  gui/main.cpp)

target_link_libraries(traffic-editor gui_lib)

set_property(TARGET traffic-editor PROPERTY ENABLE_EXPORTS 1)

install(
  TARGETS traffic-editor
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY
    plugins
  DESTINATION
    include/traffic_editor)

install(
  DIRECTORY
    include/traffic_editor
  DESTINATION
    include)

if (BUILD_TESTING)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(rmf_utils REQUIRED)

  add_subdirectory(test)

  find_file(uncrustify_config_file
    NAMES "rmf_code_style.cfg"
    PATHS "${rmf_utils_DIR}/../../../share/rmf_utils/")

  ament_uncrustify(
    ARGN include gui
    CONFIG_FILE ${uncrustify_config_file}
    MAX_LINE_LENGTH 80
    LANGUAGE CPP
  )
endif()

ament_export_include_directories(include)

ament_package()
