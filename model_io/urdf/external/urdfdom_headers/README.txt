urdfdom_headers
===========

The URDF (U-Robot Description Format) headers provides core data structure headers for URDF.

For now, the details of the URDF specifications reside on http://ros.org/wiki/urdf
  
### Build Status
[![Build Status](https://travis-ci.org/ros/urdfdom_headers.png)](https://travis-ci.org/ros/urdfdom_headers)

### Using with ROS

If you choose to check this repository out for use with ROS, be aware that the necessary ``package.xml`` is not 
included in this repo but instead is added in during the ROS release process. To emulate this, pull the appropriate
file into this repository using the following format. Be sure to replace the ALLCAPS words with the apropriate terms:

```
wget https://raw.github.com/ros-gbp/urdfdom_headers-release/debian/ROS_DISTRO/UBUNTU_DISTRO/urdfdom_headers/package.xml
```

For example:
```
wget https://raw.github.com/ros-gbp/urdfdom_headers-release/debian/hydro/precise/urdfdom_headers/package.xml
```

