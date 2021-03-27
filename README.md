[![](https://github.com/osrf/traffic_editor/workflows/ci/badge.svg)](https://github.com/osrf/traffic_editor/actions/workflows/ci.yaml)
[![](https://github.com/osrf/traffic_editor/workflows/style/badge.svg)](https://github.com/osrf/traffic_editor/actions/workflows/style.yaml)

# rmf_traffic\_editor

Welcome!

This repository has the following directories:
 * `rmf_traffic_editor`: GUI for annotating floorplans to create traffic patterns
 * `rmf_building_map_tools`: Python-based tools to use and manipulate the map files created by `rmf_traffic_editor`, such as:
   * a ROS 2 node to serve maps using `building_map_msgs`
   * translators to simulators such as Gazebo
   * translators to navigation packages such as `rmf_core`

# Installation

This repository is structured as a collection of ROS 2 packages and can be built using `colcon`.

The `rmf_building_map_tools` package requires the following Python 3 dependencies to generate worlds:

```
sudo apt install python3-shapely python3-yaml python3-requests
```
