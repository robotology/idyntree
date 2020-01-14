
# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [1.0.0] - 2020-01-14

### Added
- Added method to compute the inverse dynamics inertial parameters regressor in KinDynComputations ( https://github.com/robotology/idyntree/pull/480 ).
KinDynComputations finally reached feature parity with respect to DynamicsComputations, that will finally be removed in one of the future iDynTree feature releases.
- Added method to return the convex hull of the constraint on the projection of the center of mass (https://github.com/robotology/idyntree/pull/478).
- Added objects to deal with linear optimal control problems in the optimalcontrol library.
- Added ``OSQP`` interface via ``osqp-eigen`` in the optimalcontrol library.
- Fixed bugs in ``MultipleShooting`` solver in the optimalcontrol library.
- Added few lines of documentation in the optimalcontrol library.
- Added interface for ``ALGLIB`` and ``WORHP`` in the optimalcontrol library.
- Multiple shooting solvers can use hessians of costs and constraints in the optimalcontrol library.
- Taking into account also the sparsity pattern of constraints and dynamical system (both in jacobians and hessians) in the optimalcontrol library.
- Added visualization of vectors in the visualization library.
- Added a SolidShape helper library. This library is part of iDynTree, and is meant
to contain all the algorithms that use in some form the visual and collision geometries of the model,
and so they depend on the Assimp library to load meshes.
- Added an helper function that provides rough estimates of the inertial parameters (mass, first moments of mass,
3d inertia matrix elements) of a robot given the total mass of the robot, and its collisions shapes. While the estimates
provided are quite rough, they can be quite useful at least to provide an expected order of magnitude of the parameters,
to normalize errors or as initial points of a nonlinear optimization procedure.
- Added attitude estimator interface to estimate the orientation of an IMU, given the IMU measurements (https://github.com/robotology/idyntree/pull/516).
- Added `DiscreteExtendedKalmanFilterHelper` base class (https://github.com/robotology/idyntree/pull/516).
- Added `AttitudeMahonyFilter` implementation of an explicit formulation of passive complementary filter over quaternion groups (https://github.com/robotology/idyntree/pull/516).
- Added `AttitudeQuaternionEKF` implementation (https://github.com/robotology/idyntree/pull/516).
- Added `getWorldFrameTransform` implementation in `SimpleLeggedOdometry` class
- Added a new version of `changeFixedFrame` in `SimpleLeggedOdometry`class. This can be used to set a desired homogeneous transformation for the fixed frame
- Added `bindings` for `AttitudeMahonyFilter`, `AttitudeQuaternionEKF`, `DiscreteExtendedKalmanFilterHelper` (https://github.com/robotology/idyntree/pull/522)
- Added basic tests for the Attitude Estimator classes (https://github.com/robotology/idyntree/pull/522)
- Added `DiscreteKalmanFilterHelper` class for an implementation of a discrete, linear time-invariant Kalman Filter  (https://github.com/robotology/idyntree/pull/559)
- Added dynamic reset functionality to `DiscreteExtendedKalmanFilterHelper` class (https://github.com/robotology/idyntree/pull/553)
- Added high-level Matlab/Octave wrappers of the iDyntree bindings (https://github.com/robotology/idyntree/pull/530)
- Added bindings for the class `Span` with the name `DynamicSpan` (https://github.com/robotology/idyntree/pull/522)
- Implement `RPYRightTrivializedDerivativeRateOfChange()` and `RPYRightTrivializedDerivativeInverseRateOfChange()` into `Rotation` class
- Implement left Jacobian and left Jacobian inverse of SO(3) in Rotation class (https://github.com/robotology/idyntree/pull/562)
- Add nameIsValid attribute to iDynTree::SolidShape class.
- Add operator[] method to `iDynTree::VectorDynSize` (https://github.com/robotology/idyntree/pull/596)
- Add operator[] method to `iDynTree::VectorFixSize` (https://github.com/robotology/idyntree/pull/596)
- Implement `getTotalMass()` method for Model class
- Enable the installation of the `ModelTestUtils.h` file (https://github.com/robotology/idyntree/pull/607)
- Added `iDynTree::ModelExporter` class to export `iDynTree::Model` instances to URDF files (https://github.com/robotology/idyntree/pull/554).
- Added support in the URDF parser to correctly parse the optional name parameter of visual and collision elements.
- Added `iDynTree::ModelCalibrationHelper` to simplify loading a model from file, update its inertial parameters and exporting again to file (https://github.com/robotology/idyntree/pull/576).
- In `yarprobotstatepublisher`, add `tf-prefix` and `jointstates-topic` options for the tf prefixes and ROS topic.
* In `yarprobotstatepublisher`, add `reduced-model` optional parameter to stream only the link transformations to transform server. By default, tranformations from all the frames are streamed to the transform server.

### Changed
- The changelog has been migrated to the format described in https://keepachangelog.com/en/1.0.0/ .
- If the IDYNTREE_USES_YARP option is enabled, the minimum required version of YARP is 3.3 .
- The CMake config files are now installed in ${CMAKE_INSTALL_PREFIX}/lib/cmake/iDynTree also in Windows.
- Updated `iDynTree::ModelLoader` class to load by default models with normalized joint ordering (https://github.com/robotology/idyntree/issues/491).
- In `yarprobotstatepublisher` the model joint positions values are initialized to zero and the joint positions values are updadted in run time if the values are available in ROS topic given through `jointstates-topic` parameter.
- In `yarprobotstatepublisher`, joint size check between model joints and joints in ROS topic given through `jointstates-topic` parameter have been removed.

### Deprecated
- All the classes and methods that end in Semantics are deprecated, and will be removed in iDynTree 2.0, see https://github.com/robotology/idyntree/pull/622 for more info.
- The CMake option IDYNTREE_USES_KDL and all the classes available when enabling it are deprecated, and will be removed in iDynTree 2.0 .
- In `yarprobotstatepublisher`, the `robot` option is deprecated, and replaced by `name-prefix`.

### Fixed
- Fixed missing `DOF_ACCELLERATION` data in dynamic variable cache ordering
(https://github.com/robotology/idyntree/pull/587)
- Fixed compatibility of `Span` with SWIG bindings compilation (https://github.com/robotology/idyntree/pull/522)
- Fixed implementation of `Transform::log()` method (https://github.com/robotology/idyntree/pull/562)
- Fixed implementation of `SpatialMotionVector::exp()` method (https://github.com/robotology/idyntree/pull/562)

## [0.11.2] - 2019-12-12

### Added

- The getFrameAcc method that returns the acceleration of a frame was added to the KinDynComputations class.
  As this method takes in input every time the robot acceleration, it is computationally expensive and
  is not suitable to be used for multiple frames in a tight loop. If you need a computationally convenient
  method to access frame accelerations, please open an issue ( https://github.com/robotology/idyntree/pull/482 ).
- It is now possible to specify a non-zero bias base acceleration as input of the ForwardBiasAccKinematics function.
  This is convenient if the bias acceleration that is being computed is the bias acceleration obtained with the
  MIXED velocity representation ( https://github.com/robotology/idyntree/pull/482 ).


### Fixed

- Fixed cache invalidation bug in the getFrameBiasAcc method of KinDynComputations. The internal
  cache used by getBiasAcc was never updated even if the method setRobotState was called, so the
  getFrameBiasAcc method always returned the bias acceleration corresponding to the first call to setRobotState ( https://github.com/robotology/idyntree/pull/482 ).
- Fixed getBiasAcc method in KinDynComputations to take into account the effect of non-zero and non-parallel
  linear and angular base velocity, described in https://github.com/robotology/idyntree/issues/370 .
- Fixed compilation on 32-bit Windows ( https://github.com/robotology/idyntree/pull/506 )
- Fixed a URDF parser regression introduced in iDynTree 0.11 for which collision geometries were also loaded as visual geometries, and viceversa (https://github.com/robotology/idyntree/issues/497, https://github.com/robotology/idyntree/pull/559).
- Fixed a URDF parser regression introduced in iDynTree 0.11 for which geometry elements without origin tag were not correctly parsed (https://github.com/robotology/idyntree/issues/496, https://github.com/robotology/idyntree/pull/564).


## [0.11.1] - 2018-09-10

### Fixed

- Fix a locale-dependent floating point parsing in the URDF parser (https://github.com/robotology/idyntree/pull/475).
- Fix compatibility with SWIG 2 (https://github.com/robotology/idyntree/pull/476).
- Fix behavior of InverseKinematics if Add***Constraint method are called multiple times (https://github.com/robotology/idyntree/pull/479).


## [0.11.0] - 2018-08-31

### Added
- A new URDF parser was implemented. A side effect of the new parser is that the serialization of the links and joints of models
  parsed from URDF may change. In your code, please use name string (in place of indices) to identify joints and links, or use the
  `iDynTree::ModelLoader` class to load a Model with the specified joint serialization.
- Added library `idyntree-modelio-xml` to parse generic XML files. Support for XSD validation (when parsing from file). This library requires [Gnome libxml2](http://xmlsoft.org).
- Added `toEigen` methods for `Span` and added typedef for `Eigen` maps of vectors.
- Added some typedefs to `VectorDynsize` and `VectorFixSize` classes to enable the use of `make_span` with these objects.
- Added copy operator in `VectorDynSize` and `VectorFixSize` for `Span<double>`.
- Added method to obtain relative Jacobians sparsity pattern.
- Added method to obtain free floating Jacobians sparsity pattern.

### Changed
- iDynTree now official supports the following compilers: Microsoft Visual Studio 2015/2017, GCC >= 5.3 and clang >= 3.8 .
- The C++14 standard language is now used in the project, including in the public headers.
- [libxml2](http://xmlsoft.org) is now a required dependency. Check the docs to see how to install it.
- CMake 3.5 is now required to build iDynTree.
- Improve SpatialInertia documentation ( https://github.com/robotology/idyntree/pull/435 ).
- Changed URDF parser to use `libxml2` instead of TinyXML.

### Deprecated
- The `iDynTree/Sensors/SixAxisFTSensor.h` header has been deprecated in favor of the `iDynTree/Sensors/SixAxisForceTorqueSensor.h`.
- Constraints Jacobian now exploit sparsity pattern


## [0.10.0] - 2018-06-20

## Added

- Added the `iDynTree::Span` class (https://github.com/robotology/idyntree/pull/434), modeled after
  the C++20's `std::span` ( http://en.cppreference.com/w/cpp/container/span ). This class can be used to write
  methods and functions that access a contiguous sequence of objects in memory with a known length.
  This is extremly useful to make sure that `iDynTree` classes are easy to use regardless of which
  vector type the downstream code is using, for example if it is one of `std::vector<double>`,
  `Eigen::VectorXd` or `yarp::sig::Vector`
- In the InverseKinematics library, the Frame Constraints can now be enabled and disabled dynamically ( https://github.com/robotology/idyntree/pull/389 ).
- Addition of iDynTree::SchmittTrigger, iDynTree::ContactStateMachine and iDynTree::BipedFootContactClassifier classes for performing contact state detection using Schmitt trigger based thresholding and biped foot contact classification based on an alternate contact switching pattern reasoning over contact makes used for getting primary foot in contact
  (https://github.com/robotology/idyntree/pull/410 ,   https://github.com/robotology/idyntree/pull/411 ).
- Addition of iDynTree::GravityCompensationHelpers class for computing gravity compensation torques using accelerometer measurements (https://github.com/robotology/idyntree/pull/438)
- Initial improvement of the optimal control library ( https://github.com/robotology/idyntree/pull/442 ). See the inline documentation of the classes for more details, or open an issue
  ( https://github.com/robotology/idyntree/issues/new ) requesting documentation on some specific aspects of the optimal control library.


## Changed
- Since 0.10 release, iDynTree uses C++14 in its headers, and requires GCC >= 5 and Visual Studio >= 2015
  to compile.
- Similar to the YARP policy, any new version of iDynTree will have a consecutive minor version number.
  Development version (the one contained in the `devel` branch before release will be denoted with a
  patch version number greater than 100. The next minor release of iDynTree will be 0.11 .


## [0.8.2] - 2018-06-20

### Fixed

- In the classes devoted to the external force-torque estimation, since https://github.com/robotology/idyntree/pull/343 it is possible to have external forces that are completly estimated from external sensors such as skin sensors. If in a given submodels there are no unknown due to this, the pseudoinverse should be skipped ( https://github.com/robotology/idyntree/pull/443 ) .
- Fix compatibility with YARP 3.


## [0.8.1] - 2017-09-29

### Fixed
- The `toEigen(const SpatialMotionVector & vec)` and `toEigen(const SpatialForceVector & vec)`  inline helpers functions in the `iDynTree/Core/EigenHelpers.h` have been modified to return a **`const`** copy of their content.
While this is technically an API change, it was critical because most of the other `toEigen` methods return an `Eigen::Map` object, and users
expect to be able to use `Eigen` modifiers methods such as `toEigen(...).setZero()`. Enforcing a compilation error in this case will help to prevent subtle bugs.
As this is an `inline` function, this modification does not affect `iDynTree`'s `ABI` ( https://github.com/robotology/idyntree/pull/378 ) .
- The CMake configuration files of the release `0.8.0` were generated with the wrong version `0.7.2`.

## [0.8.0] - 2017-09-25

### Added

- This is the first version of iDynTree for which a changelog was introduced.
- The iDynTree::Model class now supports prismatic joints, using the iDynTree::PrismaticJoint class. Support for prismatic joints was added also to the URDF parser ( https://github.com/robotology/idyntree/pull/269 ).
- Classes and function to convert between chains rappresented by the Denavit Hartnberg convention and iDynTree::Model object have been implemented ( https://github.com/robotology/idyntree/pull/350 ).

### Changed

- The methods and constructors of the iDynTree::RevoluteJoint and iDynTreee::PrismaticJoint classes handling joints have been cleaned up ( https://github.com/robotology/idyntree/pull/339 ).
- The logic to enable/disable dependencies has changed. In particular, now all the dependencies (excluding the legacy dependency on KDL) that are found on the system are enabled. Users can still select manually the dependency that they want to compile using the `IDYNTREE_USES_<dep>` variables ( https://github.com/robotology/idyntree/pull/323 ).
- IPOPT has been added as an optional dependency for the Inverse Kinematics component.
- The Octave bindings have been migrated to use the exact same mex code used for the Matlab bindings ( https://github.com/robotology/idyntree/pull/305 ).
