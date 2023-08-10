^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf\_traffic\_editor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.6.2 (2023-08-10)
------------------

1.6.1 (2023-06-05)
------------------
* Switch changelogs to rst format.
* Contributors: Yadunund

1.6.0 (2022-10-05)
------------------

* Added a dispensable field for models (`#436 <https://github.com/open-rmf/rmf_traffic_editor/pull/436>`_)
* Use index based iterator to avoid yaml cpp bug (`#435 <https://github.com/open-rmf/rmf_traffic_editor/pull/435>`_)
* better vertex text size in meter-scale maps (`#431 <https://github.com/open-rmf/rmf_traffic_editor/pull/431>`_)
* Contributors: Aaron Chong, Luca Della Vedova, Morgan Quigley

1.5.1 (2022-04-20)
------------------
* update OSM tile server URL to full planet data (#430)
* fix cmake ament_index_cpp dependency
* add a github CI job for rolling
* don't double-trigger CI runs
* update qt packages for jammy github workflow
* build/install rmf_utils from source in ci_rolling
* Contributors: Morgan Quigley, Youliang Tan

1.5.0 (2022-03-13)
------------------
* fix #419 by calculating transformed x/y in image-based maps (#421)
* validate reference level index before using it (avoid a crash)
* Use C++17 for std::optional
* move all CI to galactic on ubuntu 20.04
* move the C++ style check into the 'main' CI test workflow
* Feature/render OpenStreetMap tiles (#418)
  * "new building" dialog box which asks which coordinates to use
  * use a subdirectory in ~/.cache as the OSM tile cache
  * show the tile cache size on the status bar
  * use EPSG:3857 (meters) for rendering tile maps
  * WGS84 coordinate translation
  * render OSM tiles in grayscale, so the non-tile things are easier to see
  * populate lat, lon property fields in EPSG:3857 mode
  * remove obsolete "flattened" UI stuff
  * save global coords in wgs84 but render them in epsg3857 on OSM tiles
  * GUI box to set the local CRS for sim/nav generation
  * move all RMF keys into GeoJSON props. add start/end names
  * feature_type -> rmf_type in GeoJSON properties
  * translate robot spawn point along with the world
* Handle simulation offsets for models and cameras (#408)
  * lat/lon translation behavior for models (previously, builds break when adding models due to WGS84 translation not having a "rotation" variable)
  * change naming for wgs84 model positions to lat/lon instead of x/y for consistency
  * apply offsets to the camera, so the camera view appears over the working area
  * Handle simulation offsets for models and cameras
  * Pass transform in Model constructor; pass un-transformed variables for to_yaml
  * Add check if global_transform was initialized; fix wrong computation of xy
  * parse model coordinates in wgs84 and project for viewing/editing
  * move model projection/translation to generate phase
  * set user-agent string in HTTP tile requests
  * workaround for 32-bit scrollbar overflow at extreme zoom
  * introduce a simple queuing system for tiles
  * reset zoom when loading a different filename
* flip layer images right-side up in cartesian meters mode (#405)
* Improve behavior for Cartesian maps (#401)
  * Improve behavior for Cartesian maps
  * because Cartesian maps have 1-meter units, we need to compute
  a better default scale for them. Previously it was always using
  a default scale of 0.05.
  * somewhat related, also fix the edge-select implementation so it
  spins through all edges in the map rather than just selecting
  the first edge within 10 units... that was OK when we were always
  using pixel-based maps, but now that meter-based maps are in use,
  it was just choosing the first edge that was within 10 meters of
  the click, which was a lot of edges and felt somewhat random.
  * stop drawing a 1-unit border around the scene rectangle
  * rotate vertex icons to +Y for cartesian maps
* Cartesian worlds (y=up) and steps towards using GeoPackage (#396)
  * create passthrough transform for cartesian_meters coordinate systems
  * y-flip in traffic-editor GUI for cartesian worlds; only invert Y coordinate for legacy image-based maps
  * add coordinate system files for C++ GUI
  * pass coordinate system to vertex draw, to flip text as needed
  * correct deprecated setuptools key to fix warning
  * Fix errors when building maps with lifts / crowdsim
  * draw fewer arrowheads for increased speed on very large maps
  * add speed limit parameter to lane property-editor GUI
  * add speed limit param to generated nav-graph files
  * publish lane speed limits
  * add top-level metadata for building/site params
  * load/save top level building params. Zoom->reset to center map view if you get lost.
  * assign a nonsense CRS if one doesn't exist
  * Change Legacy -> ReferenceImage throughout code
* minor usability enhancements on traffic editor (#398)
  * revert lift vertex, and prevent delete of lift vertex
  * fix wall transparency models and update readme
* Contributors: Morgan Quigley, Youliang Tan, Luca Della Vedova, Charayaphan Nakorn Boon Han

1.4.0 (2021-09-02)
------------------
* Feature/graph names and widths (`#384 <https://github.com/open-rmf/rmf_traffic_editor/pull/384>`_)
  * Graph data structure: default lane widths and a step towards `#378 <https://github.com/open-rmf/rmf_traffic_editor/pull/378>`_
* added support for ceilings with texture (`#383 <https://github.com/open-rmf/rmf_traffic_editor/pull/383>`_)
* added scaling features to wall texture (`#382 <https://github.com/open-rmf/rmf_traffic_editor/pull/382>`_)
* resolve build error on some systems with size_t namespace (`#374 <https://github.com/open-rmf/rmf_traffic_editor/pull/374>`_)
  fix build error on some compilers/systems reported in https://github.com/open-rmf/rmf/discussions/85   by adding std:: prefix to size_t
* sort list by model name, not Fuel group name (`#373 <https://github.com/open-rmf/rmf_traffic_editor/pull/373>`_)
* Feature: align vertices colinear (`#372 <https://github.com/open-rmf/rmf_traffic_editor/pull/372>`_)
* adding maintainer for buildfarm notifications (`#368 <https://github.com/open-rmf/rmf_traffic_editor/pull/368>`_)
* hotfix for `#366 <https://github.com/open-rmf/rmf_traffic_editor/pull/366>`_, avoid exploding transform for 1 fudicual (`#367 <https://github.com/open-rmf/rmf_traffic_editor/pull/367>`_)
* Minor tweak to how empty crowd_sim and lift structures are serialized in YAML (`#364 <https://github.com/open-rmf/rmf_traffic_editor/pull/364>`_)
* Contributors: Marco A. Gutiérrez, Morgan Quigley, Xiyu

1.3.0 (2021-05-14)
------------------
* Feature/display layer transforms in freefleet format (`#347 <https://github.com/open-rmf/rmf_traffic_editor/pull/347>`_)
* Feature/layer rendering palette mapping (`#344 <https://github.com/open-rmf/rmf_traffic_editor/pull/344>`_)
* Fix asset path after package renaming (`#341 <https://github.com/open-rmf/rmf_traffic_editor/pull/341>`_)
* Automatic alignment of robot-map layers to floorplans (`#340 <https://github.com/open-rmf/rmf_traffic_editor/pull/340>`_)
* Fix/ci package name (`#339 <https://github.com/open-rmf/rmf_traffic_editor/pull/339>`_)
* clarify labels on property add/delete buttons (`#326 <https://github.com/open-rmf/rmf_traffic_editor/pull/326>`_)
* handle editing multiple layers with same name. (`#328 <https://github.com/open-rmf/rmf_traffic_editor/pull/328>`_)
* fix `#324 <https://github.com/open-rmf/rmf_traffic_editor/pull/324>`_, update layer image immediately after OK button (`#327 <https://github.com/open-rmf/rmf_traffic_editor/pull/327>`_)
* Bug/add layer button not visible in new building (`#313 <https://github.com/open-rmf/rmf_traffic_editor/pull/313>`_)
* provide zoom-reset and clamp on scale factor (`#318 <https://github.com/open-rmf/rmf_traffic_editor/pull/318>`_)
* avoid crash in empty crowdsim save routine (`#312 <https://github.com/open-rmf/rmf_traffic_editor/pull/312>`_)
* rename building_map_tools (`#310 <https://github.com/open-rmf/rmf_traffic_editor/pull/310>`_)
* Rename packages and delete moved packages (`#308 <https://github.com/open-rmf/rmf_traffic_editor/pull/308>`_)
* Refactoring and Migration `#308 https://github.com/open-rmf/rmf_traffic_editor/pull/308`
* Contributors: Geoffrey Biggs, Luca Della Vedova, Morgan Quigley, youliang

1.2.0 (2021-01-05)
------------------
* Adds undo capability to a large part of the actions. (`#269 <https://github.com/osrf/traffic_editor/pull/269>`_) (`#266 <https://github.com/osrf/traffic_editor/pull/266>`_)
* Contibutors: Arjo, Morgan Quigley, Yadu
* Merge pull request `#276 <https://github.com/osrf/traffic_editor/pull/276>`_ from osrf/add_lane_vertex_snap_distance_scaling
  scale add-lane vertex snap distance correctly
* undo features
* compute the click-to-merge treshold in pixels
* scale add-lane vertex snap distance correctly
* fix to compilation when no opencv (`#272 <https://github.com/osrf/traffic_editor/pull/272>`_)
* Merge branch 'feature/undo' of https://github.com/osrf/traffic_editor into feature/undo
* Added "save" to newly updated transition entries (`#265 <https://github.com/osrf/traffic_editor/pull/265>`_)
* Merge pull request `#263 <https://github.com/osrf/traffic_editor/pull/263>`_ from osrf/dont_crash_on_new_project_crowdsim
* Merge pull request `#257 <https://github.com/osrf/traffic_editor/pull/257>`_ from Briancbn/pr-fix-disable-plugin-backwards-compatibility
* Merge pull request `#255 <https://github.com/osrf/traffic_editor/pull/255>`_ from osrf/feature/remove-plugin-option
* Configured gui to load and save proper parameters for plugins
* Using just the Lift object to propagate the plugin removal option
* Added option to remove plugins for doors on gui and building_map_tools generator
* Add GUI to traffic editor for crowd simulation configuration (`#225 <https://github.com/osrf/traffic_editor/pull/225>`_)
* Merge pull request `#249 <https://github.com/osrf/traffic_editor/pull/249>`_ from osrf/fix/lift_dialog_saving
* Add first pass of quality declarations for all packages (`#235 <https://github.com/osrf/traffic_editor/pull/235>`_)
* Contributors: Aaron Chong, Arjo Chakravarty, Chen Bainian, Geoffrey Biggs, Guoliang (Fred) Shao, Marco A. Gutiérrez, Morgan Quigley, Tian En


1.1.0 (2020-09-24)
------------------
* Focal / Ignition dome support (`#230 <https://github.com/osrf/traffic_editor/pull/230>`_)
* Adding lift operation range selection (`#220 <https://github.com/osrf/traffic_editor/pull/220>`_)
* Add field in lift dialog for initial floor, handle invalid initial floor
* Update lift display (`#216 <https://github.com/osrf/traffic_editor/pull/216>`_)
* Allowing modification on vertex coordinates (`#215 <https://github.com/osrf/traffic_editor/pull/215>`_)
* Merge pull request `#212 <https://github.com/osrf/traffic_editor/pull/212>`_ from osrf/feature/model-list-sort-script
  Feature/model list sort script
* Added helper script to sort model_list yamls
* Support for adding and recognizing lift waypoints for multi-level navigation `#201 <https://github.com/osrf/traffic_editor/pull/201>`_
* fix initial model angle, so it doesn't rotate 90 when placed (`#202 <https://github.com/osrf/traffic_editor/pull/202>`_)
* Add button to generate lift waypoints in the GUI
* Configurable texture and transparency for wall `#200 <https://github.com/osrf/traffic_editor/pull/200>`_
* fix lifts not cleared when opening another project `#196 <https://github.com/osrf/traffic_editor/pull/196>`_
* New traffic editor thumbnail generator `#191 <https://github.com/osrf/traffic_editor/pull/191>`_
* View menu option to show/hide models `#174 <https://github.com/osrf/traffic_editor/pull/174>`_
* Add add_edge shift alignment feature `#173 <https://github.com/osrf/traffic_editor/pull/173>`_
* Contributors: Aaron Chong, Chen Bainian, Geoffrey Biggs, Kevin_Skywalker, Luca Della Vedova, MakinoharaShouko, Marco A. Gutierrez, Morgan Quigley, Yadu, Yadunund, kevinskwk, methylDragon, youliang

1.0.0 (2020-06-22)
------------------
* Implement using thumbnails from installed traffic_editor_assets ament package (`#152 <https://github.com/osrf/traffic_editor/pull/152>`_)
  * Implement parsing thumbnails from assets ament package
  * Remove ExternalProject
  * Update style
  * Catch missing package error
  Co-authored-by: Marco A. Gutiérrez <spyke.me@gmail.com>
* Merge pull request `#153 <https://github.com/osrf/traffic_editor/pull/153>`_ from osrf/bug/model_orientation
  Fixed orientation of model thumbnails in the gui
* Fixed orientation of model thumbnails in the gui
* Merge pull request `#149 <https://github.com/osrf/traffic_editor/pull/149>`_ from osrf/simulation_plugin_interface
  process-flow sim plugin interface, and various other improvements
* Merge pull request `#150 <https://github.com/osrf/traffic_editor/pull/150>`_ from osrf/update_style_check
  Update style.yaml
* Added braces around for in project.cpp
* Update traffic_editor/package.xml
  Co-authored-by: Marco A. Gutiérrez <marco@openrobotics.org>
* let's not crash when loading an empty map
* remove unused unique_ptr namespace inclusion
* merging master
* Merge pull request `#148 <https://github.com/osrf/traffic_editor/pull/148>`_ from osrf/fix_crop_python_style
  merging since this is trivial (famous last words)
* python line was too long
* Merge pull request `#147 <https://github.com/osrf/traffic_editor/pull/147>`_ from MakinoharaShouko/master
  Fix not generating cropped image with namespace
* Fix not generating cropped image with namespace
* Merge pull request `#1 <https://github.com/osrf/traffic_editor/pull/1>`_ from MakinoharaShouko/crop_fix
  Fix not generating cropped image with namespace
* Fix not generating cropped image with namespace
* simplify by getting rid of pointers where possible
* since opencv is only needed for video recording, it's now optional
* hide the sim controls if there is no plugin present
* Merge pull request `#132 <https://github.com/osrf/traffic_editor/pull/132>`_ from methylDragon/ch3/author-namespaced-thumbnails
  Support Author-namedspaced Thumbnails and Revamp building_map_generator
* Merge branch 'master' into ch3/author-namespaced-thumbnails
* Correct README
* Merge pull request `#128 <https://github.com/osrf/traffic_editor/pull/128>`_ from methylDragon/ch3/migrate-traffic-editor-thumbnails
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
* Merge pull request `#130 <https://github.com/osrf/traffic_editor/pull/130>`_ from osrf/fix/model-thumbnail-names
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
* Merge pull request `#118 <https://github.com/osrf/traffic_editor/pull/118>`_ from osrf/feature/teleport-dispenser
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
* Merge pull request `#117 <https://github.com/osrf/traffic_editor/pull/117>`_ from osrf/master
  bring in medium-size surgical trolley
* Merge pull request `#116 <https://github.com/osrf/traffic_editor/pull/116>`_ from osrf/feature/surgical-trolley-med
  added thumbnail for SurgicalTrolleyMed
* added thumbnail for SurgicalTrolleyMed
* restore rotation of StorageRack thumbnail
* Merge pull request `#115 <https://github.com/osrf/traffic_editor/pull/115>`_ from osrf/master
  bring new thumbnails to dev branch
* Merge pull request `#114 <https://github.com/osrf/traffic_editor/pull/114>`_ from osrf/feature/trolley-bed-thumbnails
  Feature/trolley bed thumbnails
* copied to wrong places, replaced old thumbnails
* added thumbnails
* WIP teleporting other models for cargo pickup/dropoff
* improve nav graph following, simplify creation of non-zero graph_idx
* Merge pull request `#113 <https://github.com/osrf/traffic_editor/pull/113>`_ from osrf/master
  merge in thumbnail improvements
* Merge pull request `#112 <https://github.com/osrf/traffic_editor/pull/112>`_ from osrf/feature/more-thumbnails
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
* Merge pull request `#111 <https://github.com/osrf/traffic_editor/pull/111>`_ from osrf/fix/robot-placeholder-thumbnails
  fix model thumbnail and naming convention
* fix model thumbnail and naming convention
* Merge pull request `#110 <https://github.com/osrf/traffic_editor/pull/110>`_ from osrf/fix/thumbnail-name
  fixed bookshelf thumbnail name
* propagate unique_ptr usage to allow polymorphic compositions
* fixed bookshelf thumbnail name
* allow modifying of lanes in traffic mode and simplify renderings of bidirectional lanes.
* don't insert scenario table twice
* beginnings of sim thread
* learning about elite c++11 memory features
* working towards minimalist behavior sequencing
* Merge pull request `#108 <https://github.com/osrf/traffic_editor/pull/108>`_ from osrf/feature/new-thumbnails
  Feature/new thumbnails
* removed empty newlines
* changed back camera height
* corrected thumbnail names to point to open source gazebo models
* WIP external traffic files
* more gazebo thumbnails, compressed largge thumbnails
* add skeleton for traffic map dialog
* render traffic map names in tablewidget
* Merge pull request `#104 <https://github.com/osrf/traffic_editor/pull/104>`_ from osrf/feature/thumbnail-generation
  Feature/thumbnail generation
* save/load traffic-map references in project file
* lint :sparkles:
* added generation and merging utility scripts
* removed ros2 launch
* basic pipeline and docs added
* start of external traffic map files in GUI
* Merge pull request `#103 <https://github.com/osrf/traffic_editor/pull/103>`_ from osrf/feature/demo-assets
  added new demo asset thumbnails
* added new demo asset thumbnails
* Merge pull request `#100 <https://github.com/osrf/traffic_editor/pull/100>`_ from osrf/double_swing_doors_directions
  branch on double swing door direction for sim generation
* branch on double swing door direction for sim generation
* Merge pull request `#94 <https://github.com/osrf/traffic_editor/pull/94>`_ from osrf/static_parameter_for_models
  Static parameter for models
  Tested manually on a few worlds, looks OK
* add static param to models in GUI
* Merge pull request `#90 <https://github.com/osrf/traffic_editor/pull/90>`_ from osrf/feature/single-doors
  Feature/single doors
* abs values for motion degrees, use motion direction instead
* fix merge conflict
* Merge pull request `#81 <https://github.com/osrf/traffic_editor/pull/81>`_ from osrf/add_flattened_offsets
  XY translation of each level in a 'flattened' world generation mode
* XY translation of each level in a 'flattened' world generation mode
* Merge pull request `#80 <https://github.com/osrf/traffic_editor/pull/80>`_ from osrf/floor_holes
  Floor holes
* click selects holes first, then other polygon types
* Merge branch 'master' of ssh://github.com/osrf/traffic_editor into floor_holes
* Merge pull request `#79 <https://github.com/osrf/traffic_editor/pull/79>`_ from osrf/feature/model-elevation
  Feature/model elevation
* reverted back to using double for model::z, makes yaml parsing cleaner
* clear all fields of project when creating a new one
* added elevation/z param to model
* WIP floor holes GUI tool
* add some icons as we add a new tool for polygon-holes
* only override drawing scale if >2 fiducials are present
* more small fixes for levels without scale
* trivial: update level table after adding a level
* Merge pull request `#71 <https://github.com/osrf/traffic_editor/pull/71>`_ from osrf/fix_initial_creation_workflow
  Fix initial creation workflow. Load drawing floorplan images immediately after they are specified in the level dialog, rather than only doing it when loading the building level from YAML.
* refactor drawing loading so it can happen after level dialog also
* fix crash when no levels are present
* Merge pull request `#69 <https://github.com/osrf/traffic_editor/pull/69>`_ from osrf/create_vertex_when_starting_wall
  create a new vertex if the add-edge click is not near an existing one
* create a new vertex if the add-edge click is not near an existing one
* Merge pull request `#67 <https://github.com/osrf/traffic_editor/pull/67>`_ from osrf/create_vertex_as_needed_for_edges
  finish implementing `#63 <https://github.com/osrf/traffic_editor/pull/63>`_ to allow continuous clicks for edge creation
* finish implementing `#63 <https://github.com/osrf/traffic_editor/pull/63>`_ to allow continuous clicks for edge creation
* Merge pull request `#66 <https://github.com/osrf/traffic_editor/pull/66>`_ from osrf/click_walls_instead_of_drag
  implement part of `#63 <https://github.com/osrf/traffic_editor/pull/63>`_ so you can just keep clicking to chain vertices together
* don't automatically chain doors/measurements in edge tool
* implement part of `#63 <https://github.com/osrf/traffic_editor/pull/63>`_ so you can just keep clicking to chain vertices together
* Merge pull request `#64 <https://github.com/osrf/traffic_editor/pull/64>`_ from osrf/specify_floor_textures
  Specify floor textures
* allow specification of floor texture and scale
* Merge branch 'master' of ssh://github.com/osrf/traffic_editor into specify_floor_textures
* fix `#62 <https://github.com/osrf/traffic_editor/pull/62>`_, scale vertex click and paint doorjamb last
* add explicit polygon texture params for now
* Merge pull request `#59 <https://github.com/osrf/traffic_editor/pull/59>`_ from osrf/scenario_roi_polygon_tool
  lots of refactoring and cleanup to allow modifying scenario polygons
* lots of refactoring and cleanup to allow modifying scenario polygons
* Merge pull request `#58 <https://github.com/osrf/traffic_editor/pull/58>`_ from osrf/restore_viewport_center_and_zoom
  restore viewport translation and scale on startup
* restore viewport translation and scale on startup
* update readme
* draw traffic vertices in building coords still, for now
* Merge pull request `#53 <https://github.com/osrf/traffic_editor/pull/53>`_ from osrf/correct_thumbnail_size
  use cropped thumbnails. add four new models.
* use cropped thumbnails. add four new models.
* Merge pull request `#52 <https://github.com/osrf/traffic_editor/pull/52>`_ from osrf/separate_building_map_and_traffic_map_files
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
* Merge pull request `#51 <https://github.com/osrf/traffic_editor/pull/51>`_ from osrf/add_thumbnails
  add some new office furniture thumbnails
* add some new office furniture thumbnails
* finish propagating and refactoring buildings, so things compile again
* everything is now broken
* migrate Map to Building class
* Merge pull request `#49 <https://github.com/osrf/traffic_editor/pull/49>`_ from osrf/restore_window_position_and_size
  use qsettings to save/restore window position and size
* use qsettings to save/restore window position and size
* Merge pull request `#48 <https://github.com/osrf/traffic_editor/pull/48>`_ from osrf/add_elevation_to_level_dialog
  set level elevation via dialog box. Various cleanups.
* set level elevation via dialog box. Various cleanups.
* Merge pull request `#46 <https://github.com/osrf/traffic_editor/pull/46>`_ from osrf/calculate_and_show_scale_using_fiducials
  Calculate and show scale using fiducials
* use measurement data only on the reference level. otherwise, ficudials
* fix QGraphicsView item lookup to fix regression in selecting doors/walls
* specify reference level via map-dialog box
* redraw scene immediately after adding level
* Merge branch 'master' into calculate_and_show_scale_using_fiducials
* Merge pull request `#45 <https://github.com/osrf/traffic_editor/pull/45>`_ from osrf/fix_new_document_problems
  deal more gracefully with an empty world by not crashing
* deal more gracefully with an empty world by not crashing
* update docs to reflect new way to add levels
* map dialog for 'global' model properties
* factor level table into its own file. add meas+fiducial counts to it.
* Merge pull request `#42 <https://github.com/osrf/traffic_editor/pull/42>`_ from osrf/align_lift_layer
  Fiducials to align layers
* level alignment starting to work hooray
* WIP alignment
* working towards fiducial alignment
* Merge pull request `#41 <https://github.com/osrf/traffic_editor/pull/41>`_ from osrf/create_lifts
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
* Merge pull request `#39 <https://github.com/osrf/traffic_editor/pull/39>`_ from osrf/some_toolbar_icons
  add a few toolbar icons
* add a few toolbar icons
* Merge pull request `#38 <https://github.com/osrf/traffic_editor/pull/38>`_ from osrf/set_modified_flag
  Set modified flag
* Merge pull request `#37 <https://github.com/osrf/traffic_editor/pull/37>`_ from osrf/unify_move_tools
  unify move-vertex and move-model tools. Toolbar on top.
* ask to save changes on exit
* unify move-vertex and move-model tools. Toolbar on top.
* Merge pull request `#35 <https://github.com/osrf/traffic_editor/pull/35>`_ from osrf/levels_layers_tabs
  migrate level selection from a button bar into a tabbed table
* migrate level selection from a button bar into a tabbed table
* Merge pull request `#34 <https://github.com/osrf/traffic_editor/pull/34>`_ from osrf/editor_ui_cleanup
  Editor UI cleanup
* allow deletion of vertices and models
* refactoring model selection into its own dialog
* Merge pull request `#33 <https://github.com/osrf/traffic_editor/pull/33>`_ from osrf/add_more_models
  Add more models
* trolley bed thumbnails
* storage rack model thumbnail
* add new model thumbnails
* add storage rack thumbnails
* trivial cleanup
* Merge pull request `#32 <https://github.com/osrf/traffic_editor/pull/32>`_ from osrf/use_layer_visibility_checkboxes
  use checkboxes to specify layer visibility
* oops. optional parameter...
* Merge pull request `#31 <https://github.com/osrf/traffic_editor/pull/31>`_ from osrf/rendering_layers_controls
  Rendering layers controls
* fix compile
* rendering starting to work
* render layers
* layer yaml save/load, working towards layer dialog
* annotate YAML document with flow styles, and emit them
* add layer table and dialog for add/edit layers
* working towards selectable layers
* Merge pull request `#28 <https://github.com/osrf/traffic_editor/pull/28>`_ from osrf/generate_doors
  Generate doors
* add various door gazebo generation stuff and demo mock lift floor changes
* Merge pull request `#27 <https://github.com/osrf/traffic_editor/pull/27>`_ from osrf/add_dock_points
  add dock points and generate docking nav graph params
* add dock points and generate docking nav graph params
* Merge pull request `#25 <https://github.com/osrf/traffic_editor/pull/25>`_ from osrf/rotate_models_visually_with_discretization
  show model pixmaps rotating, with optional discretization
* show model pixmaps rotating, with optional discretization
* Merge pull request `#21 <https://github.com/osrf/traffic_editor/pull/21>`_ from osrf/spawn_robot_parameters
  robot parameters for spawning and Gazebo world generation
* create vertex parameters for spawning robots in simulation
* Merge pull request `#19 <https://github.com/osrf/traffic_editor/pull/19>`_ from osrf/redraw_after_new_file_create
  redraw after file->new, also give explicit model path for gazebo gen
* redraw after file->new, also give explicit model path for gazebo gen
* Merge pull request `#18 <https://github.com/osrf/traffic_editor/pull/18>`_ from osrf/add_install_target
  add install step in cmake
* add install step in cmake
* Merge pull request `#17 <https://github.com/osrf/traffic_editor/pull/17>`_ from osrf/ci_update_first
  update before installing in github workflow
* update before installing in github workflow
* Merge pull request `#16 <https://github.com/osrf/traffic_editor/pull/16>`_ from osrf/repository_reorganization
  Repository reorganization
* Merge branch 'master' of ssh://github.com/osrf/traffic_editor into repository_reorganization
* grand reorganization as colcon-buildable packages for ros2 integration
* Contributors: Aaron, Aaron Chong, MakinoharaShouko, Morgan Quigley, Yadu, Yadunund, methylDragon
