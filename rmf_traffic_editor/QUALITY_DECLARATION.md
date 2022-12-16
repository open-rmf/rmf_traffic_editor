This document is a declaration of software quality for the `rmf_traffic_editor` package, based on the guidelines in [REP-2004](https://www.ros.org/reps/rep-2004.html).

# `rmf_traffic_editor` Quality Declaration

The package `rmf_traffic_editor` claims to be in the **Quality Level 4** category.

Below are the rationales, notes, and caveats for this claim, organized by each requirement listed in the [Package Requirements for Quality Level 4 in REP-2004](https://www.ros.org/reps/rep-2004.html).

## Version Policy [1]

### Version Scheme [1.i]

`rmf_traffic_editor` uses `semver` according to the recommendation for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#versioning).

### Version Stability [1.ii]

`rmf_traffic_editor` is at a stable version, i.e. `>= 1.0.0`.
The current version can be found in its [package.xml](package.xml), and its change history can be found in its [CHANGELOG](CHANGELOG.rst).

### Public API Declaration [1.iii]

All symbols in the installed headers are considered part of the public API.

All installed headers are in the `include` directory of the package.
Headers in any other folders are not installed and are considered private.

### API Stability Policy [1.iv]

`rmf_traffic_editor` will not break public API within a major version number.

### ABI Stability Policy [1.v]

`rmf_traffic_editor` will not break public ABI within a major version number.

### API and ABI Stability Within a Released ROS Distribution [1.vi]

`rmf_traffic_editor` will not break public API or ABI within a released ROS distribution, i.e. no major releases into the same ROS distribution once that ROS distribution is released.

## Change Control Process [2]

`rmf_traffic_editor` follows the recommended guidelines for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#package-requirements).

### Change Requests [2.i]

`rmf_traffic_editor` requires that all changes occur through a pull request.

### Contributor Origin [2.ii]

`rmf_traffic_editor` does not require a confirmation of contributor origin.

### Peer Review Policy [2.iii]

All pull requests must have at least 1 peer review.

### Continuous Integration [2.iv]

All pull requests must pass CI on all platforms supported by RMF.
The CI checks only that the package builds.
The most recent CI results can be seen on [the workflow page](https://github.com/osrf/rmf_traffic_editor/actions).

### Documentation Policy [2.v]

All pull requests must resolve related documentation changes before merging.

## Documentation [3]

### Feature Documentation [3.i]

`rmf_traffic_editor` provides usage documentation in its [README file](README.md).

### Public API Documentation [3.ii]

`rmf_traffic_editor` does not document its public API.

### License [3.iii]

The license for `rmf_traffic_editor` is Apache 2.0, the type is declared in the [package.xml](package.xml) manifest file, and a full copy of the license is in the repository level [LICENSE](../LICENSE) file.

### Copyright Statement [3.iv]

The copyright holders each provide a statement of copyright in each source code file in `rmf_traffic_editor`.

### Quality declaration document [3.v]

This quality declaration is linked in the [README file](README.md).

This quality declaration has not been externally peer-reviewed and is not registered on any Level 4 lists.

## Testing [4]

### Feature Testing [4.i]

`rmf_traffic_editor` does not have feature tests.

### Public API Testing [4.ii]

`rmf_traffic_editor` does not have API tests.

### Coverage [4.iii]

`rmf_traffic_editor` does not track coverage statistics.

### Performance [4.iv]

`rmf_traffic_editor` does not test performance.

### Linters and Static Analysis [4.v]

`rmf_traffic_editor` does not use the standard linters and static analysis tools for its CMake code to ensure it follows the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#linters).

## Dependencies [5]

### Direct Runtime ROS Dependencies [5.i]

`rmf_traffic_editor` has no direct runtime ROS dependencies.

### Optional Direct Runtime ROS Dependencies [5.ii]

`rmf_traffic_editor` has no optional runtime ROS dependencies.

### Direct Runtime non-ROS Dependency [5.iii]

Below are the required direct runtime non-ROS dependencies of `rmf_traffic_editor` and their evaluations.

#### yaml-cpp

The [`yaml-cpp` library](https://github.com/jbeder/yaml-cpp) is assumed to be **Quality Level 3** due to its wide use, provided documentation, use of testing, and version number above 1.0.0.

#### libqt5-concurrent

`libqt5-concurrent` is widely-used third-party software for building graphical applications.
Due to its wide use, documentation, and testing, it is assumed to be **Quality Level 3**.

#### libqt5-widgets

`libqt5-widgets` is widely-used third-party software for building graphical applications.
Due to its wide use, documentation, and testing, it is assumed to be **Quality Level 3**.

## Platform Support [6]

### Target platforms [6.i]

`rmf_traffic_editor` does not support all of the tier 1 platforms as described in [REP-2000](https://www.ros.org/reps/rep-2000.html#support-tiers).
`rmf_traffic_editor` supports ROS Eloquent.

## Security [7]

### Vulnerability Disclosure Policy [7.i]

This package conforms to the Vulnerability Disclosure Policy in [REP-2006](https://www.ros.org/reps/rep-2006.html).
