cmake_minimum_required(VERSION 3.5)
project(building_sim_ign)

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

find_package(ignition-cmake2 REQUIRED)
ign_find_package(ignition-gazebo4 REQUIRED)
set(IGN_GAZEBO_VER 4)
ign_find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER 1)
ign_find_package(ignition-common3 REQUIRED)
set(IGN_COMMON_VER 3)
ign_find_package(ignition-math4 REQUIRED)
ign_find_package(sdformat9 REQUIRED)
set(IGN_MATH_VER 4)

find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rmf_fleet_msgs REQUIRED)
find_package(rmf_door_msgs REQUIRED)
find_package(building_sim_common REQUIRED)

include(GNUInstallDirs)

#add_library(rmf_sim_utils SHARED src/utils.cpp)

#target_include_directories(rmf_sim_utils
#  PUBLIC
#    ${sdformat9_INCLUDE_DIRS}
#)

#target_link_libraries(rmf_sim_utils
#PUBLIC
#ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
#${sdformat9_LIBRARIES}
#)

add_library(slotcar SHARED ${PROJECT_SOURCE_DIR}/src/slotcar.cpp)

target_link_libraries(slotcar
  PUBLIC
  #rmf_sim_utils
    ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
    ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
    ${building_sim_common_LIBRARIES}
    ${rmf_fleet_msgs_LIBRARIES}
    ${rclcpp_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${tf2_ros_LIBRARIES}
)

target_include_directories(slotcar
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    ${building_sim_common_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${rmf_fleet_msgs_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
)

# These lines are taken out because of this issue: https://github.com/ament/ament_cmake/issues/158
#ament_target_dependencies(slotcar
#  rmf_sim_utils
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

target_include_directories(door
  PUBLIC
    ${GAZEBO_INCLUDE_DIRS}
    ${building_sim_common_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${rmf_door_msgs_INCLUDE_DIRS}
)

target_link_libraries(door
  PUBLIC
  #rmf_sim_utils
    ${rclcpp_LIBRARIES}
    ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
    ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
    ${building_sim_common_LIBRARIES}
    ${rmf_door_msgs_LIBRARIES}
    ${rmf_fleet_msgs_LIBRARIES}
    #${geometry_msgs_LIBRARIES}
    #${tf2_ros_LIBRARIES}
)

#ament_target_dependencies(door
#  "rmf_fleet_msgs"
#  "rclcpp"
#  "ignition-gazebo${IGN_GAZEBO_VER}"
#  #"gazebo_dev"
#  #"gazebo_ros"
#  "rmf_door_msgs"
#)

###############################
# install stuff
###############################

install(
  TARGETS slotcar door
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

ament_package()
