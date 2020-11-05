cmake_minimum_required(VERSION 3.5)
project(building_ignition_plugins)

set (CMAKE_AUTOMOC ON)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ignition-cmake2 QUIET)
if (NOT ignition-cmake2_FOUND)
  message("ignition-cmake not found, skipping ignition gazebo plugins")
  return()
endif()
ign_find_package(ignition-gazebo4 QUIET)
if (NOT ignition-gazebo4_FOUND)
  message("Wrong ignition-gazebo version found. Plugins require ignition-gazebo4 (dome)")
  # To generate a setup.bash and avoid CMake warnings
  ament_package()
  return()
endif()
set(IGN_GAZEBO_VER 4)
ign_find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER 1)
ign_find_package(ignition-common3 REQUIRED)
set(IGN_COMMON_VER 3)
ign_find_package(ignition-math6 REQUIRED)
set(IGN_MATH_VER 6)
ign_find_package(ignition-gui4 REQUIRED)
set(IGN_GUI_VER 4)
ign_find_package(ignition-msgs6 REQUIRED)
set(IGN_MSGS_VER 6)
ign_find_package(ignition-transport9 REQUIRED)
set(IGN_TRANSPORT_VER 9)
ign_find_package(sdformat10 REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(rmf_door_msgs REQUIRED)
find_package(rmf_lift_msgs REQUIRED)
find_package(building_map_msgs REQUIRED)
find_package(building_sim_common REQUIRED)
find_package (Qt5
  COMPONENTS
    Core
    Qml
    Quick
  REQUIRED
)
find_package(menge QUIET)

include(GNUInstallDirs)

add_library(slotcar SHARED ${PROJECT_SOURCE_DIR}/src/slotcar.cpp)

ament_target_dependencies(slotcar
  PUBLIC
    ignition-gazebo${IGN_GAZEBO_VER}
    ignition-plugin${IGN_PLUGIN_VER}
    building_sim_common
    rmf_fleet_msgs
    rclcpp
    geometry_msgs
    tf2_ros
)

target_include_directories(slotcar
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${building_sim_common_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${rmf_fleet_msgs_INCLUDE_DIRS}
    ${building_map_msgs_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
)

###############################
# door stuff
###############################

add_library(door SHARED src/door.cpp)

target_include_directories(door
  PUBLIC
    ${building_sim_common_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${rmf_door_msgs_INCLUDE_DIRS}
)

ament_target_dependencies(door
  PUBLIC
    rclcpp
    ignition-gazebo${IGN_GAZEBO_VER}
    ignition-plugin${IGN_PLUGIN_VER}
    building_sim_common
    rmf_door_msgs
    rmf_fleet_msgs
)

###############################
# lift stuff
###############################

add_library(lift SHARED src/lift.cpp)

target_include_directories(lift
  PUBLIC
    ${building_sim_common_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${rmf_door_msgs_INCLUDE_DIRS}
    ${rmf_lift_msgs_INCLUDE_DIRS}
)

ament_target_dependencies(lift
  PUBLIC
    rclcpp
    ignition-gazebo${IGN_GAZEBO_VER}
    ignition-plugin${IGN_PLUGIN_VER}
    building_sim_common
    rmf_door_msgs
    rmf_lift_msgs
    rmf_fleet_msgs
)

###############################
# crowd simulator stuff
###############################
if (menge_FOUND)
  add_library(crowd_simulator
    SHARED
    src/crowd_simulator.cpp
  )

  target_include_directories(crowd_simulator
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${building_sim_common_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${menge_INCLUDE_DIRS}
    ${IGNITION-COMMON_INCLUDE_DIRS}
  )

  ament_target_dependencies(crowd_simulator
    ignition-gazebo${IGN_GAZEBO_VER}
    ignition-plugin${IGN_PLUGIN_VER}
    rclcpp
    building_sim_common
    menge
  )

  #install
  install(
    TARGETS crowd_simulator
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  )
else (NOT menge_FOUND)
  message("menge-cmake not found, skipping crowd_simulation ignition plugins")
endif()

###############################
# toggle charging
###############################

QT5_ADD_RESOURCES(resources_RCC src/toggle_charging/toggle_charging.qrc)

add_library(toggle_charging SHARED ${headers_MOC}
  src/toggle_charging/toggle_charging.cpp
  ${resources_RCC}
)

ament_target_dependencies(toggle_charging
  ignition-gui${IGN_GUI_VER}
  ignition-msgs${IGN_MSGS_VER}
  ignition-transport${IGN_TRANSPORT_VER}
  Qt5Core
  Qt5Qml
  Qt5Quick
  rclcpp
)

target_include_directories(toggle_charging
  PUBLIC
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Qml_INCLUDE_DIRS}
  ${Qt5Quick_INCLUDE_DIRS}
)

###############################
# install stuff
###############################

install(
  TARGETS slotcar door lift toggle_charging
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

ament_package()
