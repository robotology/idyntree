kdl_format_io
=============
[![Build Status](https://travis-ci.org/traversaro/kdl_format_io.png)](https://travis-ci.org/traversaro/kdl_format_io)

Routines for import and export of different robotic description formats in the KDL library.

The library is under *ACTIVE* developement. 

This software is realized for the [CoDyCo](http://www.codyco.eu/) project.

Format support
--------------
The format currently supported are:

| File Format | Import | Export |
|-------------|--------|--------|
| [URDF](http://wiki.ros.org/urdf)        |   ✔    |   ✔    |
| [SYMORO](http://www.irccyn.ec-nantes.fr/spip.php?article601&lang=en) .par |   ✔  (only Tree type for now)  |        |


### symoro_par_import
Routine for parse a [SYMORO](http://www.irccyn.ec-nantes.fr/spip.php?article601&lang=en) .par file to create a KDL Tree.


### urdf_import
Routine for parse a URDF file to a create a KDL Tree. Forked from ROS [robot_model](http://ros.org/wiki/robot_model)
[kdl_parser](http://ros.org/wiki/kdl_parser), removing dependencies on ROS and porting it to urdfdom and console_bridge.

### urdf_export
Routine for creating a URDF file from a KDL Tree, or to update (preserving
visual,collisions,materials,...) a URDF file using the geometric and 
inertial parameters of a KDL Tree. 

### par2urdf
Utility to convert a [SYMORO](http://www.irccyn.ec-nantes.fr/spip.php?article601&lang=en) .par file into a URDF file.

Installation
------------

### orocos_kdl
The only strict dependency of kdl_format_io is the orocos kinematics dynamics library (kdl). 
It is possible to install it from the [official repository](https://github.com/orocos/orocos_kinematics_dynamics):
```bash
git clone https://github.com/orocos/orocos_kinematics_dynamics
```
If instead you use a Windows enviroment, please download this fork:
```bash
git clone https://github.com/traversaro/orocos_kinematics_dynamics
```
For example, on Linux then the installation can be completed with the following commands:
```
cd orocos_kinematics_dynamics/orocos_kdl
mkdir build 
cd build
ccmake ../
make
sudo make install
```

### urdfdom
To compile URDF support in kdl_format_io, you have to install the [urdfdom library](https://github.com/ros/urdfdom).

