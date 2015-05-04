iDynTree [![Build Status](https://travis-ci.org/robotology-playground/idyntree.svg?branch=master)](https://travis-ci.org/robotology-playground/idyntree)
===========

iDynTree is a C++ library of algorithms related to robots dynamics, specifically designed for free floating robots. 


## Installation

It is recommended to install iDynTree via the [codyco-superbuild](https://github.com/robotology/codyco-superbuild)/

### Bindings 
To compile bindings to iDynTree in several scriping languages, you should enable them using the `IDYNTREE_USES_PYTHON`, `IDYNTREE_USES_LUA`, `IDYNTREE_USES_MATLAB` CMake options.

Then, properly accessing bindings to iDynTree can require some additional steps. 
#### Python
You should add to the `PYTHONPATH` enviromental variable the install path of the `iDynTree.py` file.
For a typical installation of the `codyco-superbuild`, this will require adding to the `.bashrc` a line similar to this:
~~~
export PYTHONPATH=$PYTHONPATH:~/src/codyco-superbuild/build/install/lib/python2.7/dist-packages/
~~~

#### Dependencies
##### Build dependencies
- [CMake](http://www.cmake.org)
- [YCM](https://github.com/robotology/ycm)

##### Core
- [Kinematics and Dynamics Library](https://github.com/orocos/orocos_kinematics_dynamics)
- [Eigen](http://eigen.tuxfamily.org)
- [YCM](https://github.com/robotology/ycm)
- [urdfdom](https://github.com/ros/urdfdom)

##### Optional 
- [YARP](https://github.com/robotology/yarp)
- [ICUB](https://github.com/robotology/icub-main)




### Reference paper
A paper describing the algorithms implemented in iDynTree and their use in a real world scenario can be downloaded [here](http://journal.frontiersin.org/article/10.3389/frobt.2015.00006/abstract) . If you're going to use this library for your work, please quote it within any resulting publication: 
~~~
F. Nori, S. Traversaro, J. Eljaik, F. Romano, A. Del Prete, D. Pucci "iCub whole-body control through force regulation on rigid non-coplanar contacts", Frontiers in Robotics and AI, 2015.
~~~

The bibtex code for including this citation is provided:
~~~
@ARTICLE{10.3389/frobt.2015.00006,
  AUTHOR={Nori, Francesco  and  Traversaro, Silvio  and  Eljaik, Jorhabib  and  Romano, Francesco  and  Del Prete, Andrea  and  Pucci, Daniele},
  TITLE={iCub Whole-body Control through Force Regulation on Rigid Noncoplanar Contacts}, 
  JOURNAL={Frontiers in Robotics and AI},      
  VOLUME={2},      
  YEAR={2015},      
  NUMBER={6},     
  URL={http://www.frontiersin.org/humanoid_robotics/10.3389/frobt.2015.00006/abstract},       
  DOI={10.3389/frobt.2015.00006},      
  ISSN={2296-9144}}
~~~
