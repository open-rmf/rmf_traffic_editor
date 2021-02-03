^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package building_map_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2021-01-06)
------------------
* Ign rtf optimizations and GUI plugins (`#248 <https://github.com/osrf/traffic_editor/issues/248>`_)
* Merge pull request `#257 <https://github.com/osrf/traffic_editor/issues/257>`_ from Briancbn/pr-fix-disable-plugin-backwards-compatibility
  Fix door, lift plugin disable options backwards compatibility
* Merge pull request `#255 <https://github.com/osrf/traffic_editor/issues/255>`_ from osrf/feature/remove-plugin-option
  Feature/remove plugin option
* Three lines of code to double the RTF, :sparkles:
* Makes lifts static if plugins are not required
* Door plugins option parsing correctly now
* Using just the Lift object to propagate the plugin removal option
* Added option to remove plugins for doors on gui and building_map_tools generator
* Fix namespace for rmf charging plugin (`#253 <https://github.com/osrf/traffic_editor/issues/253>`_)
* Textures/additional (`#244 <https://github.com/osrf/traffic_editor/issues/244>`_)
* Implement battery drain and recharge for slotcars (`#242 <https://github.com/osrf/traffic_editor/issues/242>`_)
* Implement animation switching in crowd simulation (`#238 <https://github.com/osrf/traffic_editor/issues/238>`_)
* Add first pass of quality declarations for all packages (`#235 <https://github.com/osrf/traffic_editor/issues/235>`_)
* Building_crowdsim for generating the navmesh file and required configuration files for menge (`#224 <https://github.com/osrf/traffic_editor/issues/224>`_)
* Add building_crowdsim to generate navmesh and config files for crowd simulation (`#224 <https://github.com/osrf/traffic_editor/pull/224>`_)
* Contributors: Aaron Chong, Chen Bainian, Geoffrey Biggs, Guoliang (Fred) Shao, Luca Della Vedova, Marco A. Gutiérrez, Morgan Quigley, Rushyendra Maganty, Valerie


1.1.0 (2020-09-24)
------------------
* Implement model visibility toggling (`#226 <https://github.com/osrf/traffic_editor/issues/226>`_)
* Adding lift operation range selection (`#220 <https://github.com/osrf/traffic_editor/issues/220>`_)
* Add field in lift dialog for initial floor, handle invalid initial floor
* added inertia for lift cabin platform (`#217 <https://github.com/osrf/traffic_editor/issues/217>`_)
* Model counts to be owned by Building, and passed to each Level `#211 <https://github.com/osrf/traffic_editor/issues/211>`_
* Support for adding and recognizing lift waypoints for multi-level navigatio `#201 <https://github.com/osrf/traffic_editor/issues/201>`_ 
* Fixed the wrong naming of reference_floor (`#209 <https://github.com/osrf/traffic_editor/issues/209>`_)
* Fixing building failure when fiducial values are integers (`#208 <https://github.com/osrf/traffic_editor/issues/208>`_)
* Generate models at the correct Z height of their level `#207 <https://github.com/osrf/traffic_editor/issues/207>`_
* Configurable texture and transparency for wall `#200 <https://github.com/osrf/traffic_editor/issues/200>`_ 
* Added wall tex in building map generation
* Handle situation when lifts key is not present `#188 <https://github.com/osrf/traffic_editor/issues/188>`_ 
* Ignition plugins and modularization of doors and slotcar `#138 <https://github.com/osrf/traffic_editor/issues/138>`_
* Adding lift pluting for ignition `#171 <https://github.com/osrf/traffic_editor/issues/171>`_
* Implement automatic lift waypoint setting
* Implement convenience script (`#185 <https://github.com/osrf/traffic_editor/issues/185>`_)
* Implement model downloader: a script to assist in model downloading without needing to also build the worldfile from a specified traffic_editor file. `#180 <https://github.com/osrf/traffic_editor/issues/180>`_
* Contributors: Aaron Chong, Chen Bainian, Geoffrey Biggs, Kevin_Skywalker, Luca Della Vedova, MakinoharaShouko, Morgan Quigley, kevinskwk, methylDragon, youliang

1.0.0 (2020-06-22)
------------------
* merging master
* Merge pull request `#134 <https://github.com/osrf/traffic_editor/issues/134>`_ from methylDragon/ch3/hotfix-nonetype-pit-crew-bug
  Make pit_crew robust against missing author names
* Make pit_crew robust against missing author names
* Merge pull request `#133 <https://github.com/osrf/traffic_editor/issues/133>`_ from osrf/fix/pit-crew-deps
  Adding instructions to install pit_crew dependency
* lint :skull:
* lint :sparkles:
* Merge pull request `#132 <https://github.com/osrf/traffic_editor/issues/132>`_ from methylDragon/ch3/author-namespaced-thumbnails
  Support Author-namedspaced Thumbnails and Revamp building_map_generator
* Fix build and import bug
* Unify building_map_generators
  With argparse and pit_crew!
* Merge branch 'master' into ch3/migrate-traffic-editor-thumbnails
* Merge branch 'master' into ch3/author-namespaced-thumbnails
* Merge pull request `#129 <https://github.com/osrf/traffic_editor/issues/129>`_ from methylDragon/ch3/pit-crew
  Unleash the pit_crew!
* Merge pull request `#131 <https://github.com/osrf/traffic_editor/issues/131>`_ from osrf/bug/building_map_server
  Fix coordinate frame of lifts and doors in building_map_server
* Implement easier logging init
* Implement lower param
* Implement cache rebuilding option
* Extend pit_crew to support ign directories
* Fix assertion bug
* Clarify assertion
* Implement use_dir_as_name
* Add model config param
* Add usage examples
* Implement dry run downloads
* Clarify docstrings, add swag, lower param, and asserts
* Use namedtuples
* Merge branch 'master' into ch3/migrate-traffic-editor-thumbnails
* Minor fixes
* Fixed format
* Fixed coordinate system for lifts and doors populated in BuildingMap msg
* Fix import bug
* Allow shutil to fail gracefully
* Include author name when returning downloadable models (for now)
* Fix capitalisation bug
* Refine logger formatting
* Fix set bug
* Implement input sanitisation
* Clarify log strings
* Fix import bug
* Reorder __all_\_ for parity with code
* Neaten description
* Unleash the pit_crew!
* Merge pull request `#127 <https://github.com/osrf/traffic_editor/issues/127>`_ from osrf/fix/door_elevation
  Fix/door elevation
* Fixed code style
* Fixed elevation of doors and floors in simulation
* Merge pull request `#122 <https://github.com/osrf/traffic_editor/issues/122>`_ from osrf/fix/building_map_server
  Fix/building map server
* Motion range of doors specified in radians
* Vertices of lift doors populated
* Format fixes
* Lift skeleton
* Lift skeleton
* Fixed format
* doors populated in map server
* Merge pull request `#118 <https://github.com/osrf/traffic_editor/issues/118>`_ from osrf/feature/teleport-dispenser
  Feature/teleport dispenser
* append number to model names only if not unique
* Merge remote-tracking branch 'origin' into external_traffic_map_files
* bugfix in hole generator in building_map_tools
* Merge pull request `#100 <https://github.com/osrf/traffic_editor/issues/100>`_ from osrf/double_swing_doors_directions
  branch on double swing door direction for sim generation
* Merge pull request `#98 <https://github.com/osrf/traffic_editor/issues/98>`_ from osrf/camera_pose
  Add computed camera pose to ignition
* branch on double swing door direction for sim generation
* Add computed camera pose to ignition, add it to gazebo template
* Merge pull request `#96 <https://github.com/osrf/traffic_editor/issues/96>`_ from osrf/fix_normals_in_wall_meshes
  hopefully fix norm and texture indexing in wall obj files
* Merge pull request `#97 <https://github.com/osrf/traffic_editor/issues/97>`_ from osrf/fix/double-swing-door-direction
  corrected simulation double swing door direction
* corrected simulation double swing door direction
* pycodestyle
* hopefully fix norm and texture indexing in wall obj files
* Merge pull request `#94 <https://github.com/osrf/traffic_editor/issues/94>`_ from osrf/static_parameter_for_models
  Static parameter for models
  Tested manually on a few worlds, looks OK
* parse model static attribute and apply during SDF generation
* Merge pull request `#92 <https://github.com/osrf/traffic_editor/issues/92>`_ from osrf/add_shapely_dep
  Add dependency to python-shapely in package.xml
* Merge pull request `#93 <https://github.com/osrf/traffic_editor/issues/93>`_ from osrf/fix_server_scale
  Fix server to latest changes in level transform
* Fix server to latest changes in level transform
* Add dependency to python-shapely in package.xml
* Merge pull request `#91 <https://github.com/osrf/traffic_editor/issues/91>`_ from osrf/calculate_floorplan_drawing_rotations
  Calculate floorplan drawing rotations
* pycodestyle fix
* finish propagating transform changes through
* finish estimating fiducial alignments, including rotation
* WIP dealing with buildings with some rotated floorplans
* Merge pull request `#90 <https://github.com/osrf/traffic_editor/issues/90>`_ from osrf/feature/single-doors
  Feature/single doors
* added flip motion direction for swing doors
* WIP open/close positions flipped at -90 and -1
* Merge branch 'master' into feature/single-doors
* single door types work, WIP get the gazebo plugins synced up for door.cpp
* Merge pull request `#89 <https://github.com/osrf/traffic_editor/issues/89>`_ from osrf/add_gazebo_plugins
  add gazebo plugins used by building_map_tools generators
* add gazebo plugins used by building_map_tools generators
* WIP fixing direction, angle of opening
* parsing hinged and sliding single doors
* handle parsing of single doors
* Merge pull request `#86 <https://github.com/osrf/traffic_editor/issues/86>`_ from osrf/fix/missing-fiducials-tag
  check if key in dict first
* lint :skull:
* check if key in dict first
* bugfix: somewhat more robust yaml parsing
* don't generate wall mesh tags if there aren't any walls
* Merge pull request `#85 <https://github.com/osrf/traffic_editor/issues/85>`_ from osrf/toggle_floors_gui_plugin
  generate params for toggle-floor GUI plugin
* generate params for toggle-floor GUI plugin
* Merge pull request `#84 <https://github.com/osrf/traffic_editor/issues/84>`_ from osrf/fix_doors
  fix wall collision bitmask and door scaling issues
* fix wall collision bitmask and door scaling issues
* Merge pull request `#83 <https://github.com/osrf/traffic_editor/issues/83>`_ from osrf/ignition_generator
  Ignition generator
* pass options list through for gz/ign tweaks
* fix gz template to actually be gazebo stuff
* Merge pull request `#82 <https://github.com/osrf/traffic_editor/issues/82>`_ from osrf/ign
  merge
* fix merge conflict
* use share path rather than file-relative path
* create the actual ignition generator, whoops
* add options flags to generator call chain for ign/gz
* Merge pull request `#81 <https://github.com/osrf/traffic_editor/issues/81>`_ from osrf/add_flattened_offsets
  XY translation of each level in a 'flattened' world generation mode
* Brighten up doors
* Remove redundant ambient tag
* Fix world name (hence ign gazebo plugins)
* Add xml tag to generated world
* Remove namespaced name from plugin
* Fix door plugin name for ignition
* First series of hacks for ignition compatibility
* XY translation of each level in a 'flattened' world generation mode
* Merge pull request `#80 <https://github.com/osrf/traffic_editor/issues/80>`_ from osrf/floor_holes
  Floor holes
* use specified level elevations; don't scale by default
* instantiate floor hole polygons using Shapely
* fix pycodestyle complaint
* Merge branch 'master' of ssh://github.com/osrf/traffic_editor into floor_holes
* Merge pull request `#79 <https://github.com/osrf/traffic_editor/issues/79>`_ from osrf/feature/model-elevation
  Feature/model elevation
* lint again
* lint
* added printout to mention deprecated model z field without elevation/z
* z in yaml parsing does not need scaling
* reverted back to using double for model::z, makes yaml parsing cleaner
* Merge pull request `#77 <https://github.com/osrf/traffic_editor/issues/77>`_ from osrf/pycodestyle_action_and_fixes
  Pycodestyle action and many python style fixes
* fix various python style abominations
* Merge pull request `#76 <https://github.com/osrf/traffic_editor/issues/76>`_ from osrf/defer_scaling_in_world_generation
  Calculate scale and translation to align building levels
* calculate scale and translation to align generated building levels
* working towards using fiducials in gazebo level generation
* Merge pull request `#75 <https://github.com/osrf/traffic_editor/issues/75>`_ from osrf/simplify_floor_polygons
  simplify floor polygons to eliminate duplicate vertices
* simplify floor polygons to eliminate duplicate vertices
* Merge pull request `#74 <https://github.com/osrf/traffic_editor/issues/74>`_ from osrf/generate_wall_meshes
  Generate wall meshes
* recursive triangulation function and slight clean-up of code abominations
* WIP generating a mega-wall obj. It's way faster than primitives.
* WIP towards wall meshes instead of primitive collections
* Add thickness to wall length
* Merge pull request `#64 <https://github.com/osrf/traffic_editor/issues/64>`_ from osrf/specify_floor_textures
  Specify floor textures
* don't crash
* create new vertices as needed for the triangles cropped by concave edges
* WIP debugging triangulation holes
* allow specification of floor texture and scale
* Merge pull request `#61 <https://github.com/osrf/traffic_editor/issues/61>`_ from osrf/use_shapely_for_geometry
  Use shapely for geometry
* fix triangle winding order after intersection and camera pose
* shapely triangulation now looking OK for convex hulls.
* figuring out a path forward...
* Merge pull request `#60 <https://github.com/osrf/traffic_editor/issues/60>`_ from osrf/port_ign_changes
  Port ign changes
* Remove unused function
* Simplify ignition migration
* Merge pull request `#55 <https://github.com/osrf/traffic_editor/issues/55>`_ from osrf/update_yaml_key_names
  fix `#54 <https://github.com/osrf/traffic_editor/issues/54>`_, update yaml key names
* fix `#54 <https://github.com/osrf/traffic_editor/issues/54>`_, update yaml key names
* Merge pull request `#50 <https://github.com/osrf/traffic_editor/issues/50>`_ from osrf/initial_multilevel_sdf
  parse fiducials
* parse fiducials
* Merge pull request `#47 <https://github.com/osrf/traffic_editor/issues/47>`_ from osrf/bug/fix-dict-illegal-access
  Bug/fix dict illegal accesses
* empty array initialization instead
* remove ABOMINATION
* added None initialization and checks, in case map is really really minimal
* Merge pull request `#29 <https://github.com/osrf/traffic_editor/issues/29>`_ from osrf/tweak_door_limits
  Tweaking limits on doors so they can close
* Merge pull request `#31 <https://github.com/osrf/traffic_editor/issues/31>`_ from osrf/rendering_layers_controls
  Rendering layers controls
* rendering starting to work
* Merge pull request `#30 <https://github.com/osrf/traffic_editor/issues/30>`_ from osrf/fix_orientation_on_unidirectional_edge_conversion
  Fix orientation on unidirectional edge conversion
* use brain
* fix regression on bidirectional->unidirectional orientation constraints
* Improve inertial parameters
* Tweaking limits on doors so they can close
* Merge pull request `#28 <https://github.com/osrf/traffic_editor/issues/28>`_ from osrf/generate_doors
  Generate doors
* add various door gazebo generation stuff and demo mock lift floor changes
* fix flake8 fixes :) and more hacking towards doors
* flake8 fixes
* Merge pull request `#27 <https://github.com/osrf/traffic_editor/issues/27>`_ from osrf/add_dock_points
  add dock points and generate docking nav graph params
* add dock points and generate docking nav graph params
* Merge pull request `#26 <https://github.com/osrf/traffic_editor/issues/26>`_ from osrf/generate_doors
  send nav graphs in building map server and more work towards doors
* fix building map server and more work towards doors
* Merge pull request `#24 <https://github.com/osrf/traffic_editor/issues/24>`_ from osrf/calculate_robot_spawn_yaw
  calculate robot heading at spawn point using nearest edge
* calculate robot heading at spawn point using nearest edge
* Merge pull request `#23 <https://github.com/osrf/traffic_editor/issues/23>`_ from osrf/output_nav_graph_dir
  output nav graphs by name into directory given as param
* output nav graphs by name into directory given as param
* Merge pull request `#21 <https://github.com/osrf/traffic_editor/issues/21>`_ from osrf/spawn_robot_parameters
  robot parameters for spawning and Gazebo world generation
* add robots when generating world
* Merge pull request `#19 <https://github.com/osrf/traffic_editor/issues/19>`_ from osrf/redraw_after_new_file_create
  redraw after file->new, also give explicit model path for gazebo gen
* redraw after file->new, also give explicit model path for gazebo gen
* Merge pull request `#16 <https://github.com/osrf/traffic_editor/issues/16>`_ from osrf/repository_reorganization
  Repository reorganization
* calculate texture paths using ament magic
* fix up server to use same yaml parser as the generators
* grand reorganization as colcon-buildable packages for ros2 integration
* Contributors: Aaron, Aaron Chong, Luca Della Vedova, Michael X. Grey, Morgan Quigley, Yadu, Yadunund, methylDragon
