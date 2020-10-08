This document is a declaration of software quality for the `building_map_tools` package, based on the guidelines in [REP-2004](https://www.ros.org/reps/rep-2004.html).

# `building_map_tools` Quality Declaration

The package `building_map_tools` claims to be in the **Quality Level 4** category.

Below are the rationales, notes, and caveats for this claim, organized by each requirement listed in the [Package Requirements for Quality Level 4 in REP-2004](https://www.ros.org/reps/rep-2004.html).

## Version Policy [1]

### Version Scheme [1.i]

`building_map_tools` uses `semver` according to the recommendation for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#versioning).

### Version Stability [1.ii]

`building_map_tools` is at a stable version, i.e. `>= 1.0.0`.
The current version can be found in its [package.xml](package.xml), and its change history can be found in its [CHANGELOG](CHANGELOG.rst).

### Public API Declaration [1.iii]

All installed Python packages are part of the public API.

### API Stability Policy [1.iv]

`building_map_tools` will not break public API within a major version number.

### ABI Stability Policy [1.v]

`building_map_tools` will not break public ABI within a major version number.

### API and ABI Stability Within a Released ROS Distribution [1.vi]

`building_map_tools` will not break public API or ABI within a released ROS distribution, i.e. no major releases into the same ROS distribution once that ROS distribution is released.

## Change Control Process [2]

`building_map_tools` follows the recommended guidelines for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#package-requirements).

### Change Requests [2.i]

`building_map_tools` requires that all changes occur through a pull request.

### Contributor Origin [2.ii]

`building_map_tools` does not require a confirmation of contributor origin.

### Peer Review Policy [2.iii]

All pull requests must have at least 1 peer review.

### Continuous Integration [2.iv]

All pull requests must pass CI on all platforms supported by RMF.

The most recent CI results can be seen on [the workflow page](https://github.com/osrf/rmf_core/actions?query=workflow%3Abuild+branch%3Amaster).

### Documentation Policy [2.v]

All pull requests must resolve related documentation changes before merging.

## Documentation [3]

### Feature Documentation [3.i]

`building_map_tools` does not provide documentation.

### Public API Documentation [3.ii]

`building_map_tools` does not document its public API.

### License [3.iii]

The license for `building_map_tools` is Apache 2.0, the type is declared in the [package.xml](package.xml) manifest file, and a full copy of the license is in the repository level [LICENSE](../LICENSE) file.

### Copyright Statement [3.iv]

Copyright statements are not provided in the source files.

### Quality declaration document [3.v]

This quality declaration is linked in the [README file](README.md).

This quality declaration has not been externally peer-reviewed and is not registered on any Level 4 lists.

## Testing [4]

### Feature Testing [4.i]

`building_map_tools` does not provide feature tests.

### Public API Testing [4.ii]

`building_map_tools` does not provide API tests.

### Coverage [4.iii]

`building_map_tools` does not track coverage statistics.

### Performance [4.iv]

`building_map_tools` does not test performance.

### Linters and Static Analysis [4.v]

`building_map_tools` does not use the standard linters and static analysis tools for its CMake code to ensure it follows the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#linters).

## Dependencies [5]

### Direct Runtime ROS Dependencies [5.i]

Below are the required direct runtime ROS dependencies of `building_map_tools` and their evaluations.

#### rclpy

`rclpy` does not declare a quality level.
It is assumed to be **Quality Level 3** based on its wide-spread use, use of change control, use of CI, and use of testing.

#### std_msgs

`std_msgs` is [**Quality Level 3**](https://github.com/ros2/common_interfaces/blob/master/std_msgs/QUALITY_DECLARATION.md).

#### building_map_msgs

`building_map_msgs` is [**Quality Level 3**](https://github.com/osrf/traffic_editor/blob/master/building_map_msgs/QUALITY_DECLARATION.md).

#### python3-requests

`python3-requests` is assumed to be at **Quality Level 3** due to its wide-spread use, documentation, CI, and testing.

#### python3-shapely

`python3-shapely` is assumed to be at **Quality Level 3** due to its wide-spread use, documentation, CI, and testing.

### Optional Direct Runtime ROS Dependencies [5.ii]

`building_map_tools` has no optional runtime ROS dependencies.

### Direct Runtime non-ROS Dependency [5.iii]

`building_map_tools` has no runtime non-ROS dependencies.

## Platform Support [6]

### Target platforms [6.i]

`building_map_tools` does not support all of the tier 1 platforms as described in [REP-2000](https://www.ros.org/reps/rep-2000.html#support-tiers).
`building_map_tools` supports ROS Eloquent.

## Security [7]

### Vulnerability Disclosure Policy [7.i]

This package conforms to the Vulnerability Disclosure Policy in [REP-2006](https://www.ros.org/reps/rep-2006.html).
