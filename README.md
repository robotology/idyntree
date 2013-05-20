kdl_urdf 
=============
Routines for URDF import and *export* in KDL library.

The library is under *ACTIVE* developement. 

This software is realized for the [CoDyCo](http://www.codyco.eu/) project.

kdl_import
----------
Routine for parse a URDF file to a create a KDL Tree. Forked from ROS [robot_model](http://ros.org/wiki/robot_model)
[kdl_parser](http://ros.org/wiki/kdl_parser), removing dependencies on ROS and porting it to urdfdom and console_bridge.

kdl_export
----------
Routine for creating a URDF file from a KDL Tree, or to update (preserving
visual,collisions,materials,...) a URDF file using the geometric and 
inertial parameters of a KDL Tree. 
