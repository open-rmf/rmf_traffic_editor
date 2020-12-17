cmake_minimum_required(VERSION 3.5)
project(building_sim_common)

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
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(rmf_door_msgs REQUIRED)
find_package(rmf_lift_msgs REQUIRED)
find_package(building_map_msgs REQUIRED)
find_package(menge QUIET)

include(GNUInstallDirs)

add_library(slotcar_common SHARED ${PROJECT_SOURCE_DIR}/src/slotcar_common.cpp)

ament_target_dependencies(slotcar_common
    Eigen3
    rmf_fleet_msgs
    building_map_msgs
    rclcpp
    std_msgs
    geometry_msgs
    tf2_ros
)

target_include_directories(slotcar_common
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${EIGEN3_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

###############################
# door stuff
###############################

add_library(door_common SHARED src/door_common.cpp)

ament_target_dependencies(door_common
    rmf_fleet_msgs
    rclcpp
    rmf_door_msgs
)

target_include_directories(door_common
 PUBLIC
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

###############################
# lift stuff
###############################

add_library(lift_common SHARED src/lift_common.cpp)

ament_target_dependencies(lift_common
   rmf_fleet_msgs
   rclcpp
   rmf_door_msgs
   rmf_lift_msgs
)


target_include_directories(lift_common
 PUBLIC
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

###############################
# crowd simulator stuff
###############################
if (menge_FOUND)
  add_library(crowd_simulator_common
    SHARED
    src/crowd_simulator_common.cpp  
  )

  target_include_directories(crowd_simulator_common
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
      ${menge_INCLUDE_DIRS}
  )
  
  ament_target_dependencies(crowd_simulator_common
    menge
    rclcpp
  )

  #crowd_simulation_common_install
  ament_export_targets(crowd_simulator_common HAS_LIBRARY_TARGET)
  install(
    TARGETS crowd_simulator_common
    EXPORT crowd_simulator_common
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
  )
else(NOT menge_FOUND)
  message("menge-cmake not found, skipping crowd_simulation plugins")
endif()

###############################
# install stuff
###############################
ament_export_include_directories(include)

ament_export_targets(slotcar_common HAS_LIBRARY_TARGET)
ament_export_targets(door_common HAS_LIBRARY_TARGET)
ament_export_targets(lift_common HAS_LIBRARY_TARGET)

ament_export_dependencies(Eigen3)

install(
  TARGETS slotcar_common 
  EXPORT slotcar_common
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  TARGETS door_common 
  EXPORT door_common 
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  TARGETS lift_common
  EXPORT lift_common 
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(
  DIRECTORY include/building_sim_common
  DESTINATION include
)

ament_package()
