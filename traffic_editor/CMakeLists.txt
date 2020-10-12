cmake_minimum_required(VERSION 3.10.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

project(traffic_editor)

find_package(yaml-cpp REQUIRED)
find_package(ament_cmake REQUIRED)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
# set(CMAKE_VERBOSE_MAKEFILE TRUE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wshadow -Wextra")

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
  set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

find_package(Qt5 COMPONENTS Widgets Concurrent REQUIRED)
find_package(ignition-plugin1 QUIET COMPONENTS all)
find_package(ignition-common3 QUIET)
find_package(ament_index_cpp REQUIRED)

# OpenCV is an optional dependency, only needed if you want to make videos
# which is only really needed if you're using a process simulation plugin
find_package(OpenCV QUIET)

include_directories(.)
include_directories(gui)
include_directories(include)
include_directories(${ament_index_cpp_INCLUDE_DIRS})

set(traffic_editor_sources
  gui/add_param_dialog.cpp
  gui/building.cpp
  gui/building_dialog.cpp
  gui/building_level.cpp
  gui/building_level_dialog.cpp
  gui/building_level_table.cpp
  gui/edge.cpp
  gui/editor.cpp
  gui/editor_model.cpp
  gui/fiducial.cpp
  gui/layer.cpp
  gui/layer_dialog.cpp
  gui/level.cpp
  gui/lift.cpp
  gui/lift_dialog.cpp
  gui/lift_door.cpp
  gui/lift_table.cpp
  gui/main.cpp
  gui/map_view.cpp
  gui/model.cpp
  gui/model_dialog.cpp
  gui/param.cpp
  gui/polygon.cpp
  gui/preferences_dialog.cpp
  gui/preferences_keys.cpp
  gui/project.cpp
  gui/project_dialog.cpp
  gui/scenario.cpp
  gui/scenario_dialog.cpp
  gui/scenario_level.cpp
  gui/scenario_table.cpp
  gui/table_list.cpp
  gui/traffic_table.cpp
  gui/traffic_map.cpp
  gui/traffic_map_dialog.cpp
  gui/vertex.cpp
  gui/yaml_utils.cpp
  resources/resource.qrc
)

set(traffic_editor_libs
  Qt5::Widgets
  Qt5::Concurrent
  yaml-cpp
  ${ament_index_cpp_LIBRARIES}
  ${OpenCV_LIBS}
)

if (ignition-plugin1_VERSION)
  set(traffic_editor_sources
    ${traffic_editor_sources}
    gui/sim_thread.cpp
  )
  set(traffic_editor_libs
    ${traffic_editor_libs}
    ignition-plugin1::core
    ignition-plugin1::loader
    ignition-common3::core
  )
endif()

add_executable(traffic-editor ${traffic_editor_sources})
target_link_libraries(traffic-editor ${traffic_editor_libs})

if (ignition-plugin1_VERSION)
  # sadly this if() block has to come after add_executable(), but
  # the if() block determining the sources has to go before add_executable()
  target_compile_definitions(traffic-editor PUBLIC "HAS_IGNITION_PLUGIN")
endif()

if (OpenCV_VERSION)
  target_compile_definitions(traffic-editor PUBLIC "HAS_OPENCV")
endif()

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


ament_export_include_directories(include)

ament_package()
