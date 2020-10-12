cmake_minimum_required(VERSION 3.5)
project(building_gazebo_plugins)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
set(CMAKE_AUTOMOC ON)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(OpenCV REQUIRED )
find_package(gazebo_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(rmf_door_msgs REQUIRED)
find_package(rmf_lift_msgs REQUIRED)
find_package(building_map_msgs REQUIRED)
find_package(building_sim_common REQUIRED)
find_package(menge QUIET)

# TODO this is a dependency of building_sim_common, it shouldn't be needed
find_package(Eigen3 REQUIRED)

include(GNUInstallDirs)

add_library(slotcar SHARED ${PROJECT_SOURCE_DIR}/src/slotcar.cpp)

ament_target_dependencies(slotcar
  Eigen3
  building_sim_common
  rmf_fleet_msgs
  rclcpp
  gazebo_ros
  std_msgs
  geometry_msgs
  tf2_ros
  building_map_msgs
)

target_include_directories(slotcar
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${EIGEN3_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

###############################
# door stuff
###############################

add_library(door SHARED src/door.cpp)

ament_target_dependencies(door
  building_sim_common
  rmf_fleet_msgs
  rclcpp
  gazebo_ros
  rmf_door_msgs
)

target_include_directories(door
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${GAZEBO_INCLUDE_DIRS}
)

###############################
# lift stuff
###############################

add_library(lift SHARED src/lift.cpp)

ament_target_dependencies(lift
    building_sim_common
    rmf_fleet_msgs
    rclcpp
    gazebo_ros
    rmf_door_msgs
    rmf_lift_msgs
)

target_include_directories(lift
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${GAZEBO_INCLUDE_DIRS}
)

###############################
# toggle floor stuff
###############################

add_library(toggle_floors SHARED src/toggle_floors.cpp)

ament_target_dependencies(toggle_floors
    Qt5
    gazebo_ros
    rmf_fleet_msgs
    rclcpp
)

target_include_directories(toggle_floors
  PUBLIC
    ${GAZEBO_INCLUDE_DIRS}
    #${building_sim_common_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
)

###############################
# toggle charging
###############################

add_library(toggle_charging SHARED src/toggle_charging.cpp)

ament_target_dependencies(toggle_charging
    Qt5
    gazebo_ros
    rmf_fleet_msgs
    rclcpp
)

target_include_directories(toggle_charging
  PUBLIC
    ${GAZEBO_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
)


###############################
# thumbnail generation stuff
###############################

add_library(thumbnail_generator SHARED src/thumbnail_generator.cpp)

target_include_directories(thumbnail_generator
  PUBLIC
    ${GAZEBO_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(thumbnail_generator
  PUBLIC
    ${OpenCV_LIBS}
)

###############################
# crowd simulator stuff
###############################
if (menge_FOUND)
  add_library(crowd_simulator
    SHARED
      src/crowd_simulator.cpp
  )

  ament_target_dependencies(crowd_simulator
    building_sim_common
    rclcpp
    menge
  )

  target_include_directories(crowd_simulator
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      ${GAZEBO_INCLUDE_DIRS}
      ${menge_INCLUDE_DIRS}
      ${building_sim_common_INCLUDE_DIRS}   
  )

  #install
  install(
    TARGETS crowd_simulator
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  )
else (NOT menge_FOUND)
  message("menge-cmake not found, skipping crowd_simulation gazebo plugins")
endif()

###############################
# install stuff
###############################

install(
  TARGETS slotcar door lift toggle_floors toggle_charging thumbnail_generator
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

ament_package()
