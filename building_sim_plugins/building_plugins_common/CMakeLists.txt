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

include(GNUInstallDirs)

add_library(slotcar_common SHARED ${PROJECT_SOURCE_DIR}/src/slotcar_common.cpp)

target_link_libraries(slotcar_common
  PUBLIC
    Eigen3::Eigen
    ${rmf_fleet_msgs_LIBRARIES}
    ${building_map_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${tf2_ros_LIBRARIES}
)

target_include_directories(slotcar_common
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${EIGEN3_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${rmf_fleet_msgs_INCLUDE_DIRS}
    ${building_map_msgs_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
)

###############################
# door stuff
###############################

add_library(door_common SHARED src/door_common.cpp)

target_link_libraries(door_common
 PUBLIC
   ${rmf_fleet_msgs_LIBRARIES}
   ${rclcpp_LIBRARIES}
   ${rmf_door_msgs_LIBRARIES}
)

target_include_directories(door_common
 PUBLIC
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
   ${rmf_fleet_msgs_INCLUDE_DIRS}
   ${rmf_door_msgs_INCLUDE_DIRS}
)

###############################
# lift stuff
###############################

add_library(lift_common SHARED src/lift_common.cpp)

target_link_libraries(lift_common
 PUBLIC
   ${rmf_fleet_msgs_LIBRARIES}
   ${rclcpp_LIBRARIES}
   ${rmf_door_msgs_LIBRARIES}
   ${rmf_lift_msgs_LIBRARIES}
)

target_include_directories(lift_common
 PUBLIC
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
   ${rmf_fleet_msgs_INCLUDE_DIRS}
   ${rmf_door_msgs_INCLUDE_DIRS}
   ${rmf_lift_msgs_INCLUDE_DIRS}
)

###############################
# install stuff
###############################
ament_export_dependencies(Eigen3)
ament_export_include_directories(include)
ament_export_libraries(slotcar_common)
ament_export_libraries(door_common)
ament_export_libraries(lift_common)


install(
  TARGETS slotcar_common door_common lift_common #building_sim_utils slotcar door toggle_floors
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
  DIRECTORY include/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

ament_package()
