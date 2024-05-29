cmake_minimum_required(VERSION 3.5)

project(rmf_traffic_editor_assets LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY
  assets/thumbnails
  DESTINATION share/${PROJECT_NAME}
)

