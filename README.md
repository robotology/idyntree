iDynTree [![Build Status](https://travis-ci.org/robotology-playground/idyntree.svg?branch=master)](https://travis-ci.org/robotology-playground/idyntree)
===========

iDynTree is a C++ library of algorithms related to robots dynamics, specifically designed for free floating robots.


## Installation
iDynTree is developed in the context of the [CoDyCo](www.codyco.eu) European Project. For this reason it is usually installed throught the [codyco-superbuild](https://github.com/robotology/codyco-superbuild), an easy way to download, compile and install the CoDyCo software on multiple operating systems, using the [CMake](www.cmake.org) build system and its extension [YCM](http://robotology.github.io/ycm). For more informations on the superbuild concept, please check [YCM documentation](http://robotology.github.io/ycm/gh-pages/master/index.html#superbuild).  

If you are not interested in installing all the CoDyCo software it is still possible to install iDynTree without installing the rest of the CoDyCo software. For this reason a superbuild to install iDynTree and all its dependencies is available at:
https://github.com/robotology-playground/iDynTree-superbuild . Please check the README of that repo for information on how to install iDynTree and all its dependencies. 

### Bindings
To compile bindings to iDynTree in several scriping languages, you should enable them using the `IDYNTREE_USES_PYTHON`, `IDYNTREE_USES_LUA`, `IDYNTREE_USES_MATLAB` CMake options.

Several examples for using the bindigs are available in https://github.com/robotology-playground/idyntree/blob/master/doc/geometric_classes.md .

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

##### Core
- [Kinematics and Dynamics Library](https://github.com/orocos/orocos_kinematics_dynamics)
- [Eigen](http://eigen.tuxfamily.org)
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
