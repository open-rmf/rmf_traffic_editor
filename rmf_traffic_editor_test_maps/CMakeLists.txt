cmake_minimum_required(VERSION 3.5)

project(rmf_traffic_editor_test_maps)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  maps/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

file(GLOB_RECURSE traffic_editor_paths "maps/*.building.yaml")

foreach(path ${traffic_editor_paths})

  # Get the output world name
  string(REGEX REPLACE "\\.[^.]*\.[^.]*$" "" no_extension_path ${path})
  string(REGEX MATCH "[^\/]+$" world_name  ${no_extension_path})

  set(map_path ${path})
  set(output_world_name ${world_name})
  set(output_dir ${CMAKE_CURRENT_BINARY_DIR}/maps/${output_world_name})
  set(output_world_path ${output_dir}/${output_world_name}.world)
  set(output_model_dir ${output_dir}/models)

  # first, generate the world
  message("BUILDING WORLDFILE WITH COMMAND: ros2 run rmf_building_map_tools building_map_generator gazebo ${map_path} ${output_world_path} ${output_model_dir}")
  if (NO_DOWNLOAD_MODELS)
    add_custom_command(
      DEPENDS ${map_path}
      COMMAND ros2 run rmf_building_map_tools building_map_generator gazebo ${map_path} ${output_world_path} ${output_model_dir}
      OUTPUT ${output_world_path}
    )
  else()
    message("DOWNLOADING MODELS WITH COMMAND: ros2 run rmf_building_map_tools building_map_model_downloader ${map_path}")
    add_custom_command(
      DEPENDS ${map_path}
      COMMAND ros2 run rmf_building_map_tools building_map_generator gazebo ${map_path} ${output_world_path} ${output_model_dir}
      COMMAND ros2 run rmf_building_map_tools building_map_model_downloader ${map_path}
      OUTPUT ${output_world_path}
    )
  endif()

  add_custom_target(generate_${output_world_name} ALL
    DEPENDS ${output_world_path}
  )

  install(
    DIRECTORY ${output_dir}
    DESTINATION share/${PROJECT_NAME}/maps
  )

endforeach()
