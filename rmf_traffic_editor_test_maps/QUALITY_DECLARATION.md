This document is a declaration of software quality for the `rmf_traffic_editor_test_maps` package, based on the guidelines in [REP-2004](https://www.ros.org/reps/rep-2004.html).

# `rmf_traffic_editor_test_maps` Quality Declaration

The package `rmf_traffic_editor_test_maps` claims to be in the **Quality Level 3** category.
The main rationale for this claim its its status as a collection of demonstration map files.

Below are the detailed rationales, notes, and caveats for this claim, organized by each requirement listed in the [Package Requirements for Quality Level 3 in REP-2004](https://www.ros.org/reps/rep-2004.html).

## Version Policy [1]

### Version Scheme [1.i]

`rmf_traffic_editor_test_maps` uses `semver` according to the recommendation for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#versioning).

### Version Stability [1.ii]

`rmf_traffic_editor_test_maps` is at a stable version, i.e. `>= 1.0.0`.
The current version can be found in its [package.xml](package.xml), and its change history can be found in its [CHANGELOG](CHANGELOG.rst).

### Public API Declaration [1.iii]

`rmf_traffic_editor_test_maps` does not have a public API.

### API Stability Policy [1.iv]

`rmf_traffic_editor_test_maps` does not have a public API.

### ABI Stability Policy [1.v]

`rmf_traffic_editor_test_maps` does not have a public API.

### API and ABI Stability Within a Released ROS Distribution [1.vi]

`rmf_traffic_editor_test_maps` does not have a public API.

## Change Control Process [2]

`rmf_traffic_editor_test_maps` follows the recommended guidelines for ROS Core packages in the [ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/#package-requirements).

### Change Requests [2.i]

`rmf_traffic_editor_test_maps` requires that all changes occur through a pull request.

### Contributor Origin [2.ii]

`rmf_traffic_editor_test_maps` does not require a confirmation of contributor origin.

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

`rmf_traffic_editor_test_maps` is documented in the [parent repository's README.md file](https://github.com/osrf/traffic_editor/blob/master/README.md).

### Public API Documentation [3.ii]

`rmf_traffic_editor_test_maps` does not have a public API.

### License [3.iii]

The license for `rmf_traffic_editor_test_maps` is Apache 2.0, the type is declared in the [package.xml](package.xml) manifest file, and a full copy of the license is in the repository level [LICENSE](../LICENSE) file.

There are no source files that are currently copyrighted in this package so files are not checked for abbreviated license statements.

### Copyright Statement [3.iv]

There are no copyrighted source files in this package.

### Quality declaration document [3.v]

This quality declaration is linked in the [README file](README.md).

This quality declaration has not been externally peer-reviewed and is not registered on any Level 3 lists.

## Testing [4]

### Feature Testing [4.i]

`rmf_traffic_editor_test_maps` is a package providing strictly resource files and therefore does not require associated tests.

### Public API Testing [4.ii]

`rmf_traffic_editor_test_maps` is a package providing strictly resource files and therefore does not require associated tests.

### Coverage [4.iii]

`rmf_traffic_editor_test_maps` is a package providing strictly resource files and therefore does not require associated tests.

### Performance [4.iv]

`rmf_traffic_editor_test_maps` is a package providing strictly resource files and therefore does not require associated tests.

### Linters and Static Analysis [4.v]

`rmf_traffic_editor_test_maps` has no files that require linting or static analysis.

## Dependencies [5]

### Direct Runtime ROS Dependencies [5.i]

`rmf_traffic_editor_test_maps` does not have any required direct runtime ROS dependencies.

### Optional Direct Runtime ROS Dependencies [5.ii]

`rmf_traffic_editor_test_maps` does not have any optional direct runtime ROS dependencies.

### Direct Runtime non-ROS Dependency [5.iii]

`rmf_traffic_editor_test_maps` does not have any runtime non-ROS dependencies.

## Platform Support [6]

As a pure resource package, `rmf_traffic_editor_test_maps` supports all of the tier 1 platforms as described in [REP-2000](https://www.ros.org/reps/rep-2000.html#support-tiers), but does not currently test each change against all of them.

## Security [7]

### Vulnerability Disclosure Policy [7.i]

This package conforms to the Vulnerability Disclosure Policy in [REP-2006](https://www.ros.org/reps/rep-2006.html).
