# traffic\_editor

Welcome!

This repository has the following directories:
 * `traffic_editor`: GUI for annotating floorplans to create traffic patterns
 * `building_map_msgs`: messages that can carry these traffic plans
 * `building_map_tools`: Python-based tools to use and manipulate the map files created by `traffic_editor`, such as:
   * a ROS 2 node to serve maps using `building_map_msgs`
   * translators to simulators such as Gazebo
   * translators to navigation packages such as `rmf_core`
