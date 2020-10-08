This document is a declaration of software quality for the `building_map_msgs` package, based on the guidelines in [REP-2004](https://www.ros.org/reps/rep-2004.html).

# `building_map_msgs` Quality Declaration

The package `building_map_msgs` claims to be in the **Quality Level 3** category.

Below are the rationales, notes, and caveats for this claim, organized by each requirement listed in the [Package Requirements for Quality Level 3 in REP-2004](https://www.ros.org/reps/rep-2004.html).

## Version Policy [1]

### Version Scheme [1.i]

`building_map_msgs` uses `semver` according to the recommendation for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#versioning).

### Version Stability [1.ii]

`building_map_msgs` is at a stable version, i.e. `>= 1.0.0`.
The current version can be found in its [package.xml](package.xml), and its change history can be found in its [CHANGELOG](CHANGELOG.rst).

### Public API Declaration [1.iii]

All message definition files located the `msg` directory and service definition files located in the `srv` directory are considered part of the public API.

### API Stability Within a Released ROS Distribution [1.iv]/[1.vi]

`building_map_msgs` will not break public API within a released ROS distribution, i.e. no major releases once the ROS distribution is released.

### ABI Stability Within a Released ROS Distribution [1.v]/[1.vi]

`building_map_msgs` does not contain any C or C++ code and therefore will not affect ABI stability.

## Change Control Process [2]

`building_map_msgs` follows the recommended guidelines for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#package-requirements).

### Change Requests [2.i]

`building_map_msgs` requires that all changes occur through a pull request.

### Contributor Origin [2.ii]

`building_map_msgs` does not require a confirmation of contributor origin.

### Peer Review Policy [2.iii]

All pull requests must have at least 1 peer review.

### Continuous Integration [2.iv]

All pull requests must pass CI on all platforms supported by RMF.
The CI checks only that the package builds.
The most recent CI results can be seen on [the workflow page](https://github.com/osrf/traffic_editor/actions).

### Documentation Policy [2.v]

All pull requests must resolve related documentation changes before merging.

## Documentation [3]

### Feature Documentation [3.i]

`building_map_msgs` has basic comments in the message and service definition files, but no list of messages, services, or usage guide is provided.
New messages and services require their own documentation in order to be added.

### Public API Documentation [3.ii]

`building_map_msgs` has embedded API documentation, but it is not currently hosted.

### License [3.iii]

The license for `building_map_msgs` is Apache 2.0, the type is declared in the [package.xml](package.xml) manifest file, and a full copy of the license is in the repository level [LICENSE](../LICENSE) file.

There are no source files that are currently copyrighted in this package so files are not checked for abbreviated license statements.

### Copyright Statement [3.iv]

There are no copyrighted source files in this package.

### Quality declaration document [3.v]

This quality declaration is linked in the [README file](README.md).

This quality declaration has not been externally peer-reviewed and is not registered on any Level 3 lists.

## Testing [4]

### Feature Testing [4.i]

`building_map_msgs` is a package providing strictly message and service definitions and therefore does not require associated tests.

### Public API Testing [4.ii]

`building_map_msgs` is a package providing strictly message and service definitions and therefore does not require associated tests.

### Coverage [4.iii]

`building_map_msgs` is a package providing strictly message and service definitions and therefore has no coverage requirements.

### Performance [4.iv]

`building_map_msgs` is a package providing strictly message and service definitions and therefore has no performance requirements.

### Linters and Static Analysis [4.v]

`building_map_msgs` does not use the standard linters and static analysis tools for its generated C++ and Python code to ensure it follows the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#linters).

## Dependencies [5]

### Direct Runtime ROS Dependencies [5.i]

`building_map_msgs` has the following runtime ROS dependencies.

#### builtin\_interfaces

`builtin_interfaces` is at [**Quality Level 3**](https://github.com/ros2/rcl_interfaces/tree/master/builtin_interfaces/QUALITY_DECLARATION.md)

#### rosidl\_default_runtime

`rosidl_default_runtime` is at [**Quality Level 3**](https://github.com/ros2/rosidl_defaults/tree/master/rosidl_default_runtime/QUALITY_DECLARATION.md)

#### geometry_msgs

`geometry_msgs` is [**Quality Level 3**](https://github.com/ros2/common_interfaces/blob/master/geometry_msgs/QUALITY_DECLARATION.md).

### Optional Direct Runtime ROS Dependencies [5.ii]

`building_map_msgs` does not have any optional direct runtime ROS dependencies.

### Direct Runtime non-ROS Dependency [5.iii]

`building_map_msgs` does not have any runtime non-ROS dependencies.

## Platform Support [6]

As a pure message and service definitions package, `building_map_msgs` supports all of the tier 1 platforms as described in [REP-2000](https://www.ros.org/reps/rep-2000.html#support-tiers), but does not currently test each change against all of them.

## Security [7]

### Vulnerability Disclosure Policy [7.i]

This package conforms to the Vulnerability Disclosure Policy in [REP-2006](https://www.ros.org/reps/rep-2006.html).
