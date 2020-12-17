^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package traffic_editor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Upcoming release
------------------
* Adds undo capability to a large part of the actions. (`#269 <https://github.com/osrf/traffic_editor/pull/269>`_) (`#266 <https://github.com/osrf/traffic_editor/pull/266>`_)
* Contibutors: Arjo, Morgan Quigley, Yadu

1.1.0 (2020-09-24)
------------------
* Focal / Ignition dome support (`#230 <https://github.com/osrf/traffic_editor/issues/230>`_)
* Adding lift operation range selection (`#220 <https://github.com/osrf/traffic_editor/issues/220>`_)
* Add field in lift dialog for initial floor, handle invalid initial floor
* Update lift display (`#216 <https://github.com/osrf/traffic_editor/issues/216>`_)
* Allowing modification on vertex coordinates (`#215 <https://github.com/osrf/traffic_editor/issues/215>`_)
* Merge pull request `#212 <https://github.com/osrf/traffic_editor/issues/212>`_ from osrf/feature/model-list-sort-script
  Feature/model list sort script
* Added helper script to sort model_list yamls
* Support for adding and recognizing lift waypoints for multi-level navigation `#201 <https://github.com/osrf/traffic_editor/issues/201>`_
* fix initial model angle, so it doesn't rotate 90 when placed (`#202 <https://github.com/osrf/traffic_editor/issues/202>`_)
* Add button to generate lift waypoints in the GUI
* Configurable texture and transparency for wall `#200 <https://github.com/osrf/traffic_editor/issues/200>`_
* fix lifts not cleared when opening another project `#196 <https://github.com/osrf/traffic_editor/issues/196>`_ 
* New traffic editor thumbnail generator `#191 <https://github.com/osrf/traffic_editor/issues/191>`_
* View menu option to show/hide models `#174 <https://github.com/osrf/traffic_editor/issues/174>`_
* Add add_edge shift alignment feature `#173 <https://github.com/osrf/traffic_editor/issues/173>`_
* Contributors: Aaron Chong, Chen Bainian, Geoffrey Biggs, Kevin_Skywalker, Luca Della Vedova, MakinoharaShouko, Marco A. Gutierrez, Morgan Quigley, Yadu, Yadunund, kevinskwk, methylDragon, youliang

1.0.0 (2020-06-22)
------------------
* Implement using thumbnails from installed traffic_editor_assets ament package (`#152 <https://github.com/osrf/traffic_editor/issues/152>`_)
  * Implement parsing thumbnails from assets ament package
  * Remove ExternalProject
  * Update style
  * Catch missing package error
  Co-authored-by: Marco A. Gutiérrez <spyke.me@gmail.com>
* Merge pull request `#153 <https://github.com/osrf/traffic_editor/issues/153>`_ from osrf/bug/model_orientation
  Fixed orientation of model thumbnails in the gui
* Fixed orientation of model thumbnails in the gui
* Merge pull request `#149 <https://github.com/osrf/traffic_editor/issues/149>`_ from osrf/simulation_plugin_interface
  process-flow sim plugin interface, and various other improvements
* Merge pull request `#150 <https://github.com/osrf/traffic_editor/issues/150>`_ from osrf/update_style_check
  Update style.yaml
* Added braces around for in project.cpp
* Update traffic_editor/package.xml
  Co-authored-by: Marco A. Gutiérrez <marco@openrobotics.org>
* let's not crash when loading an empty map
* remove unused unique_ptr namespace inclusion
* merging master
* Merge pull request `#148 <https://github.com/osrf/traffic_editor/issues/148>`_ from osrf/fix_crop_python_style
  merging since this is trivial (famous last words)
* python line was too long
* Merge pull request `#147 <https://github.com/osrf/traffic_editor/issues/147>`_ from MakinoharaShouko/master
  Fix not generating cropped image with namespace
* Fix not generating cropped image with namespace
* Merge pull request `#1 <https://github.com/osrf/traffic_editor/issues/1>`_ from MakinoharaShouko/crop_fix
  Fix not generating cropped image with namespace
* Fix not generating cropped image with namespace
* simplify by getting rid of pointers where possible
* since opencv is only needed for video recording, it's now optional
* hide the sim controls if there is no plugin present
* Merge pull request `#132 <https://github.com/osrf/traffic_editor/issues/132>`_ from methylDragon/ch3/author-namespaced-thumbnails
  Support Author-namedspaced Thumbnails and Revamp building_map_generator
* Merge branch 'master' into ch3/author-namespaced-thumbnails
* Correct README
* Merge pull request `#128 <https://github.com/osrf/traffic_editor/issues/128>`_ from methylDragon/ch3/migrate-traffic-editor-thumbnails
  Migrate thumbnails to traffic_editor_assets repo
* Fix build and import bug
* Revert default directory and make directories if they don't exist
  Also make it less fragile by allowing expansion of the home shortcut "~"
* Add dependency on buiding_map_tools
  In order to ensure that pit_crew is accessible!
* Unify building_map_generators
  With argparse and pit_crew!
* Pit-crewify thumbnail_generators
* thumbnails::yeet()
  Let's try this again..
* Merge branch 'master' into ch3/migrate-traffic-editor-thumbnails
* Merge pull request `#130 <https://github.com/osrf/traffic_editor/issues/130>`_ from osrf/fix/model-thumbnail-names
  Fix/model thumbnail names
* Corrected thumbnail for PotatoChipChair
* Changed the name for model and thumbnail Table
* Retarget thumbnail search path to ~/.traffic_editor
* Implement git clone on build
* avoid deadlock
* adding debugging drawing hooks to simulation plugin interface
* osrf repo
* migrate behavior stuff into plugins, out of main tree
* render mixed lane colors in a predictable z-stack
* remove logging from the core traffic-editor, do it in plugins
* option to release reserved lanes during waiting behavior node
* adjust mutex: sim proceeds while video frame is writing to disk
* helper function to retrieve model instances
* WIP simplifying internal API and removing YAML scripting nonsense
* add load function to configure simulation interface from yaml
* epic restructuring of include files to allow a plugin interface for sim
* Merge pull request `#118 <https://github.com/osrf/traffic_editor/issues/118>`_ from osrf/feature/teleport-dispenser
  Feature/teleport dispenser
* learn cmake
* WIP trying to bring in ignition-plugin
* log simulations to csv
* models name instances can be edited, and saved
* print less to the console
* added teleport dispenser ingestor thumbnails, same as robot placeholder
* allow editing of model instance name
* on startup, restore editor to previous level
* WIP process flow animation machinery
* Merge pull request `#117 <https://github.com/osrf/traffic_editor/issues/117>`_ from osrf/master
  bring in medium-size surgical trolley
* Merge pull request `#116 <https://github.com/osrf/traffic_editor/issues/116>`_ from osrf/feature/surgical-trolley-med
  added thumbnail for SurgicalTrolleyMed
* added thumbnail for SurgicalTrolleyMed
* restore rotation of StorageRack thumbnail
* Merge pull request `#115 <https://github.com/osrf/traffic_editor/issues/115>`_ from osrf/master
  bring new thumbnails to dev branch
* Merge pull request `#114 <https://github.com/osrf/traffic_editor/issues/114>`_ from osrf/feature/trolley-bed-thumbnails
  Feature/trolley bed thumbnails
* copied to wrong places, replaced old thumbnails
* added thumbnails
* WIP teleporting other models for cargo pickup/dropoff
* improve nav graph following, simplify creation of non-zero graph_idx
* Merge pull request `#113 <https://github.com/osrf/traffic_editor/issues/113>`_ from osrf/master
  merge in thumbnail improvements
* Merge pull request `#112 <https://github.com/osrf/traffic_editor/issues/112>`_ from osrf/feature/more-thumbnails
  added new thumbnails for hospital environment
* added new thumbnails for hospital environment
* WIP smarter NPC motions...
* clean up compiler warnings
* add string interpolation and a signaling method
* set vertex label red if selected. try to fix github build workflow
* checkboxes for show/hide internal traffic lanes. sim starts paused.
* use opencv for video recording
* loop at end of behavior schedule
* rotate models to face the direction of travel
* path traversal starting to work
* basic a* planner seems ok
* WIP agent planning
* WIP scenario non-robot animation
* WIP towards beginnings of 2d model scripting
* load images concurrently on all CPU cores
* Merge pull request `#111 <https://github.com/osrf/traffic_editor/issues/111>`_ from osrf/fix/robot-placeholder-thumbnails
  fix model thumbnail and naming convention
* fix model thumbnail and naming convention
* Merge pull request `#110 <https://github.com/osrf/traffic_editor/issues/110>`_ from osrf/fix/thumbnail-name
  fixed bookshelf thumbnail name
* propagate unique_ptr usage to allow polymorphic compositions
* fixed bookshelf thumbnail name
* allow modifying of lanes in traffic mode and simplify renderings of bidirectional lanes.
* don't insert scenario table twice
* beginnings of sim thread
* learning about elite c++11 memory features
* working towards minimalist behavior sequencing
* Merge pull request `#108 <https://github.com/osrf/traffic_editor/issues/108>`_ from osrf/feature/new-thumbnails
  Feature/new thumbnails
* removed empty newlines
* changed back camera height
* corrected thumbnail names to point to open source gazebo models
* WIP external traffic files
* more gazebo thumbnails, compressed largge thumbnails
* add skeleton for traffic map dialog
* render traffic map names in tablewidget
* Merge pull request `#104 <https://github.com/osrf/traffic_editor/issues/104>`_ from osrf/feature/thumbnail-generation
  Feature/thumbnail generation
* save/load traffic-map references in project file
* lint :sparkles:
* added generation and merging utility scripts
* removed ros2 launch
* basic pipeline and docs added
* start of external traffic map files in GUI
* Merge pull request `#103 <https://github.com/osrf/traffic_editor/issues/103>`_ from osrf/feature/demo-assets
  added new demo asset thumbnails
* added new demo asset thumbnails
* Merge pull request `#100 <https://github.com/osrf/traffic_editor/issues/100>`_ from osrf/double_swing_doors_directions
  branch on double swing door direction for sim generation
* branch on double swing door direction for sim generation
* Merge pull request `#94 <https://github.com/osrf/traffic_editor/issues/94>`_ from osrf/static_parameter_for_models
  Static parameter for models
  Tested manually on a few worlds, looks OK
* add static param to models in GUI
* Merge pull request `#90 <https://github.com/osrf/traffic_editor/issues/90>`_ from osrf/feature/single-doors
  Feature/single doors
* abs values for motion degrees, use motion direction instead
* fix merge conflict
* Merge pull request `#81 <https://github.com/osrf/traffic_editor/issues/81>`_ from osrf/add_flattened_offsets
  XY translation of each level in a 'flattened' world generation mode
* XY translation of each level in a 'flattened' world generation mode
* Merge pull request `#80 <https://github.com/osrf/traffic_editor/issues/80>`_ from osrf/floor_holes
  Floor holes
* click selects holes first, then other polygon types
* Merge branch 'master' of ssh://github.com/osrf/traffic_editor into floor_holes
* Merge pull request `#79 <https://github.com/osrf/traffic_editor/issues/79>`_ from osrf/feature/model-elevation
  Feature/model elevation
* reverted back to using double for model::z, makes yaml parsing cleaner
* clear all fields of project when creating a new one
* added elevation/z param to model
* WIP floor holes GUI tool
* add some icons as we add a new tool for polygon-holes
* only override drawing scale if >2 fiducials are present
* more small fixes for levels without scale
* trivial: update level table after adding a level
* Merge pull request `#71 <https://github.com/osrf/traffic_editor/issues/71>`_ from osrf/fix_initial_creation_workflow
  Fix initial creation workflow. Load drawing floorplan images immediately after they are specified in the level dialog, rather than only doing it when loading the building level from YAML.
* refactor drawing loading so it can happen after level dialog also
* fix crash when no levels are present
* Merge pull request `#69 <https://github.com/osrf/traffic_editor/issues/69>`_ from osrf/create_vertex_when_starting_wall
  create a new vertex if the add-edge click is not near an existing one
* create a new vertex if the add-edge click is not near an existing one
* Merge pull request `#67 <https://github.com/osrf/traffic_editor/issues/67>`_ from osrf/create_vertex_as_needed_for_edges
  finish implementing `#63 <https://github.com/osrf/traffic_editor/issues/63>`_ to allow continuous clicks for edge creation
* finish implementing `#63 <https://github.com/osrf/traffic_editor/issues/63>`_ to allow continuous clicks for edge creation
* Merge pull request `#66 <https://github.com/osrf/traffic_editor/issues/66>`_ from osrf/click_walls_instead_of_drag
  implement part of `#63 <https://github.com/osrf/traffic_editor/issues/63>`_ so you can just keep clicking to chain vertices together
* don't automatically chain doors/measurements in edge tool
* implement part of `#63 <https://github.com/osrf/traffic_editor/issues/63>`_ so you can just keep clicking to chain vertices together
* Merge pull request `#64 <https://github.com/osrf/traffic_editor/issues/64>`_ from osrf/specify_floor_textures
  Specify floor textures
* allow specification of floor texture and scale
* Merge branch 'master' of ssh://github.com/osrf/traffic_editor into specify_floor_textures
* fix `#62 <https://github.com/osrf/traffic_editor/issues/62>`_, scale vertex click and paint doorjamb last
* add explicit polygon texture params for now
* Merge pull request `#59 <https://github.com/osrf/traffic_editor/issues/59>`_ from osrf/scenario_roi_polygon_tool
  lots of refactoring and cleanup to allow modifying scenario polygons
* lots of refactoring and cleanup to allow modifying scenario polygons
* Merge pull request `#58 <https://github.com/osrf/traffic_editor/issues/58>`_ from osrf/restore_viewport_center_and_zoom
  restore viewport translation and scale on startup
* restore viewport translation and scale on startup
* update readme
* draw traffic vertices in building coords still, for now
* Merge pull request `#53 <https://github.com/osrf/traffic_editor/issues/53>`_ from osrf/correct_thumbnail_size
  use cropped thumbnails. add four new models.
* use cropped thumbnails. add four new models.
* Merge pull request `#52 <https://github.com/osrf/traffic_editor/issues/52>`_ from osrf/separate_building_map_and_traffic_map_files
  Separate building map and traffic map files
* fix move-model bug
* allow deleting vertices from scenario
* Merge branch 'master' of ssh://github.com/osrf/traffic_editor into separate_building_map_and_traffic_map_files
* working towards adding vertices in scenarios
* change toolbar in response to edit mode. add to scenario skeleton.
* scenario save/load skeleton
* start scenario data structure
* add tabs
* project dialog box to set the building path
* starting to place building within a project...
* Merge pull request `#51 <https://github.com/osrf/traffic_editor/issues/51>`_ from osrf/add_thumbnails
  add some new office furniture thumbnails
* add some new office furniture thumbnails
* finish propagating and refactoring buildings, so things compile again
* everything is now broken
* migrate Map to Building class
* Merge pull request `#49 <https://github.com/osrf/traffic_editor/issues/49>`_ from osrf/restore_window_position_and_size
  use qsettings to save/restore window position and size
* use qsettings to save/restore window position and size
* Merge pull request `#48 <https://github.com/osrf/traffic_editor/issues/48>`_ from osrf/add_elevation_to_level_dialog
  set level elevation via dialog box. Various cleanups.
* set level elevation via dialog box. Various cleanups.
* Merge pull request `#46 <https://github.com/osrf/traffic_editor/issues/46>`_ from osrf/calculate_and_show_scale_using_fiducials
  Calculate and show scale using fiducials
* use measurement data only on the reference level. otherwise, ficudials
* fix QGraphicsView item lookup to fix regression in selecting doors/walls
* specify reference level via map-dialog box
* redraw scene immediately after adding level
* Merge branch 'master' into calculate_and_show_scale_using_fiducials
* Merge pull request `#45 <https://github.com/osrf/traffic_editor/issues/45>`_ from osrf/fix_new_document_problems
  deal more gracefully with an empty world by not crashing
* deal more gracefully with an empty world by not crashing
* update docs to reflect new way to add levels
* map dialog for 'global' model properties
* factor level table into its own file. add meas+fiducial counts to it.
* Merge pull request `#42 <https://github.com/osrf/traffic_editor/issues/42>`_ from osrf/align_lift_layer
  Fiducials to align layers
* level alignment starting to work hooray
* WIP alignment
* working towards fiducial alignment
* Merge pull request `#41 <https://github.com/osrf/traffic_editor/issues/41>`_ from osrf/create_lifts
  Lifts
* make ficudials easier to see
* add fiducial tool
* rotate lift doors correctly. Fix lift door yaml load bug
* copy lift-door checkbox matrix to data structure on OK button
* change data structure to deal with multi-door scenario on same level
* respond appropriately to edits in door table
* push lift elements into QGraphicsGroup and don't transform in dialog rendering
* live updates for lift preview
* start rendering lifts on the map using modeless dialog param updates
* save lift x,y,yaw,width,depth
* update level-door table combo box options when a door name changes
* add tables for editing doors and level-door mapping
* start working on adding lifts
* Merge pull request `#39 <https://github.com/osrf/traffic_editor/issues/39>`_ from osrf/some_toolbar_icons
  add a few toolbar icons
* add a few toolbar icons
* Merge pull request `#38 <https://github.com/osrf/traffic_editor/issues/38>`_ from osrf/set_modified_flag
  Set modified flag
* Merge pull request `#37 <https://github.com/osrf/traffic_editor/issues/37>`_ from osrf/unify_move_tools
  unify move-vertex and move-model tools. Toolbar on top.
* ask to save changes on exit
* unify move-vertex and move-model tools. Toolbar on top.
* Merge pull request `#35 <https://github.com/osrf/traffic_editor/issues/35>`_ from osrf/levels_layers_tabs
  migrate level selection from a button bar into a tabbed table
* migrate level selection from a button bar into a tabbed table
* Merge pull request `#34 <https://github.com/osrf/traffic_editor/issues/34>`_ from osrf/editor_ui_cleanup
  Editor UI cleanup
* allow deletion of vertices and models
* refactoring model selection into its own dialog
* Merge pull request `#33 <https://github.com/osrf/traffic_editor/issues/33>`_ from osrf/add_more_models
  Add more models
* trolley bed thumbnails
* storage rack model thumbnail
* add new model thumbnails
* add storage rack thumbnails
* trivial cleanup
* Merge pull request `#32 <https://github.com/osrf/traffic_editor/issues/32>`_ from osrf/use_layer_visibility_checkboxes
  use checkboxes to specify layer visibility
* oops. optional parameter...
* Merge pull request `#31 <https://github.com/osrf/traffic_editor/issues/31>`_ from osrf/rendering_layers_controls
  Rendering layers controls
* fix compile
* rendering starting to work
* render layers
* layer yaml save/load, working towards layer dialog
* annotate YAML document with flow styles, and emit them
* add layer table and dialog for add/edit layers
* working towards selectable layers
* Merge pull request `#28 <https://github.com/osrf/traffic_editor/issues/28>`_ from osrf/generate_doors
  Generate doors
* add various door gazebo generation stuff and demo mock lift floor changes
* Merge pull request `#27 <https://github.com/osrf/traffic_editor/issues/27>`_ from osrf/add_dock_points
  add dock points and generate docking nav graph params
* add dock points and generate docking nav graph params
* Merge pull request `#25 <https://github.com/osrf/traffic_editor/issues/25>`_ from osrf/rotate_models_visually_with_discretization
  show model pixmaps rotating, with optional discretization
* show model pixmaps rotating, with optional discretization
* Merge pull request `#21 <https://github.com/osrf/traffic_editor/issues/21>`_ from osrf/spawn_robot_parameters
  robot parameters for spawning and Gazebo world generation
* create vertex parameters for spawning robots in simulation
* Merge pull request `#19 <https://github.com/osrf/traffic_editor/issues/19>`_ from osrf/redraw_after_new_file_create
  redraw after file->new, also give explicit model path for gazebo gen
* redraw after file->new, also give explicit model path for gazebo gen
* Merge pull request `#18 <https://github.com/osrf/traffic_editor/issues/18>`_ from osrf/add_install_target
  add install step in cmake
* add install step in cmake
* Merge pull request `#17 <https://github.com/osrf/traffic_editor/issues/17>`_ from osrf/ci_update_first
  update before installing in github workflow
* update before installing in github workflow
* Merge pull request `#16 <https://github.com/osrf/traffic_editor/issues/16>`_ from osrf/repository_reorganization
  Repository reorganization
* Merge branch 'master' of ssh://github.com/osrf/traffic_editor into repository_reorganization
* grand reorganization as colcon-buildable packages for ros2 integration
* Contributors: Aaron, Aaron Chong, MakinoharaShouko, Morgan Quigley, Yadu, Yadunund, methylDragon
