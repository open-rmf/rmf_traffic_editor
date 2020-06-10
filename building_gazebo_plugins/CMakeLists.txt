cmake_minimum_required(VERSION 3.5)
project(building_gazebo_plugins)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
set(CMAKE_AUTOMOC ON)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
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
find_package(gazebo_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(rmf_door_msgs REQUIRED)
find_package(rmf_lift_msgs REQUIRED)
find_package(building_map_msgs REQUIRED)

include(GNUInstallDirs)

add_library(building_sim_utils SHARED src/utils.cpp)

target_include_directories(building_sim_utils
  PUBLIC
    ${GAZEBO_INCLUDE_DIRS}
)

add_library(slotcar SHARED ${PROJECT_SOURCE_DIR}/src/slotcar.cpp)

target_link_libraries(slotcar
  PUBLIC
    building_sim_utils
    ${rmf_fleet_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    ${gazebo_ros_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${tf2_ros_LIBRARIES}
    ${building_map_msgs_LIBRARIES}
)

target_include_directories(slotcar
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${GAZEBO_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${rmf_fleet_msgs_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${building_map_msgs_INCLUDE_DIRS}
)

# These lines are taken out because of this issue: https://github.com/ament/ament_cmake/issues/158
#ament_target_dependencies(slotcar
#  building_sim_utils
#  "rmf_fleet_msgs"
#  "rclcpp"
#  "gazebo_dev"
#  "gazebo_ros"
#  "geometry_msgs"
#  "tf2_ros"
#)

###############################
# door stuff
###############################

add_library(door SHARED src/door.cpp)

target_link_libraries(door
  PUBLIC
    building_sim_utils
    ${rmf_fleet_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    ${gazebo_ros_LIBRARIES}
    ${rmf_door_msgs_LIBRARIES}
)

target_include_directories(door
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${GAZEBO_INCLUDE_DIRS}
    ${gazebo_ros_INCLUDE_DIRS}
    ${rmf_fleet_msgs_INCLUDE_DIRS}
    ${rmf_door_msgs_INCLUDE_DIRS}
)

###############################
# lift stuff
###############################

# lift plugin
add_library(lift SHARED src/lift.cpp)

target_link_libraries(lift
  PUBLIC
    building_sim_utils
    ${rmf_fleet_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    ${gazebo_ros_LIBRARIES}
    ${rmf_lift_msgs_LIBRARIES}
    ${rmf_door_msgs_LIBRARIES}
)

target_include_directories(lift
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${GAZEBO_INCLUDE_DIRS}
    ${gazebo_ros_INCLUDE_DIRS}
    ${rmf_fleet_msgs_INCLUDE_DIRS}
    ${rmf_lift_msgs_INCLUDE_DIRS}
    ${rmf_door_msgs_INCLUDE_DIRS}
)

###############################
# toggle floor stuff
###############################

add_library(toggle_floors SHARED src/toggle_floors.cpp)

target_include_directories(toggle_floors
  PUBLIC
    ${GAZEBO_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
)

target_link_libraries(toggle_floors
  PUBLIC
    Qt5::Widgets
)

###############################
# install stuff
###############################

install(
  TARGETS building_sim_utils slotcar door lift toggle_floors
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

ament_package()
