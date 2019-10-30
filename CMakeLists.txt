cmake_minimum_required(VERSION 3.1.0)

project(traffic-editor)

find_package(yaml-cpp REQUIRED)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wshadow -Wextra")

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
  set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()

find_package(Qt5 COMPONENTS Widgets REQUIRED)

add_executable(traffic-editor
  edge.cpp
  editor.cpp
  editor_model.cpp
  level.cpp
  main.cpp
  map.cpp
  map_view.cpp
  model.cpp
  param.cpp
  polygon.cpp
  vertex.cpp
)

target_link_libraries(traffic-editor
  Qt5::Widgets
  yaml-cpp)
