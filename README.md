![](https://github.com/osrf/traffic_editor/workflows/build/badge.svg)
![](https://github.com/osrf/traffic_editor/workflows/style/badge.svg)

# traffic\_editor

Welcome!

This repository has the following directories:
 * `traffic_editor`: GUI for annotating floorplans to create traffic patterns
 * `building_map_msgs`: messages that can carry these traffic plans
 * `building_map_tools`: Python-based tools to use and manipulate the map files created by `traffic_editor`, such as:
   * a ROS 2 node to serve maps using `building_map_msgs`
   * translators to simulators such as Gazebo
   * translators to navigation packages such as `rmf_core`

# Installation

The GUI of `traffic-editor` can be installed and used independently from ROS 2
if desired. This process is documented in the `traffic_editor` package README.
However, building with `colcon` as part of a ROS 2 workspace allows easier
generation and use of Gazebo simulation worlds from `traffic-editor` buildings.
The `building_map_tools` package requires the following Python 3 dependencies
to generate worlds:

```
sudo apt install python3-shapely python3-yaml python3-requests
```
