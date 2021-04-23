[![](https://github.com/osrf/traffic_editor/workflows/ci/badge.svg)](https://github.com/osrf/traffic_editor/actions/workflows/ci.yaml)
[![](https://github.com/osrf/traffic_editor/workflows/style/badge.svg)](https://github.com/osrf/traffic_editor/actions/workflows/style.yaml)

# rmf_traffic\_editor

Welcome!

This repository has the following directories:
 * `rmf_traffic_editor`: GUI for annotating floorplans to create traffic patterns
 * `rmf_building_map_tools`: Python-based tools to use and manipulate the map files created by `rmf_traffic_editor`, such as:
   * `building_map_server`:  a ROS 2 node to serve maps using `rmf_building_map_msgs`
   * translators to simulators such as Gazebo
   * translators to navigation packages such as `rmf_core` (e.g. `rmf_ros2`)
   * scripts that handle downloading of gazebo models. `pit_crew`, `building_map_model_downloader`...
 * `rmf_traffic_editor_assets`: Gazebo model thumbnails, in used by `traffic_editor` GUI

# Installation

This repository is structured as a collection of ROS 2 packages and can be built using `colcon`.
For full installation of RMF, please refer to [here](https://github.com/open-rmf/rmf).

The `rmf_building_map_tools` package requires the following Python 3 dependencies to generate worlds:

```
sudo apt install python3-shapely python3-yaml python3-requests
```
