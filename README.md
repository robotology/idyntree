iDynTree [![Build Status](https://travis-ci.org/robotology/idyntree.svg?branch=maste2013-r)](https://travis-ci.org/robotology/idyntree) [![Build status](https://ci.appveyor.com/api/projects/status/1uecfmyvxb2dujt9/branch/master?svg=true)](https://ci.appveyor.com/project/robotology/idyntree/branch/master) [![License: LGPL v3](https://img.shields.io/badge/License-LGPL%20v3-blue.svg)](https://www.gnu.org/licenses/lgpl-3.0) [![License: LGPL v2](https://img.shields.io/badge/License-LGPL%20v2-blue.svg)](https://www.gnu.org/licenses/lgpl-2.1)  [![Shipping faster with ZenHub](https://raw.githubusercontent.com/ZenHubIO/support/master/zenhub-badge.png)](https://zenhub.com)
===========

iDynTree is a library of robots dynamics algorithms for control, estimation and simulation.

iDynTree is specifically designed for free-floating robots, but it is possible to use it also  with fixed-base robots.

iDynTree is written in C++ language, but thanks to [SWIG](http://www.swig.org/) it is possible to use the iDynTree algorithms in several other languages. Support and documentation is provided in particular for C++, Matlab, Python and Lua. If you are interested in using iDynTree with another programming language, feel free to [create a new issue](https://github.com/robotology/idyntree/issues/new) requesting support for it.

##  Contents
* **[Installation](#installation)**
* **[Tutorials](#tutorials)**
* **[Documentation](#documentation)**
* **[Announcements](#announcements)**
* **[Developer Documentation](#developer-documentation)**
* **[Reference Paper](#reference-paper)**
* **[Acknowledgments](#acknowledgments)**

## Installation
iDynTree is mainly developed and mantained by the [Dynamic Interaction Control research line](https://www.iit.it/research/lines/dynamic-interaction-control) at the [Italian Institute of Technology](https://www.iit.it/), as part of the [iCub project](http://www.icub.org/) .

For this reason it is usually installed through the [robotology-superbuild](https://github.com/robotology/robotology-superbuild), an easy way to download, compile and install the robotology software on multiple operating systems, using the [CMake](www.cmake.org) build system and its extension [YCM](http://robotology.github.io/ycm). For more informations on the superbuild concept, please check [YCM documentation](http://robotology.github.io/ycm/gh-pages/master/index.html#superbuild). To get iDynTree when using the `robotology-superbuild`, please enable the `ROBOTOLOGY_ENABLE_DYNAMICS` CMake option of the superbuild.

If you are not interested in installing all the robotology software it is still possible to install iDynTree without installing the rest of the robotology software, and please read the rest of the Readme for more info on this.

### Dependencies
iDynTree requires few external libraries. At the first configuration, the build system of iDynTree enables the use of the dependencies that it finds in the system,
but you can manually make sure that iDynTree searches or ignores a given dependency by enabling or disabling the `IDYNTREE_USES_<DEP>` CMake options.

##### Build dependencies
- [CMake](http://www.cmake.org)

##### Required
- [Eigen](http://eigen.tuxfamily.org)
- [Libxml2](http://xmlsoft.org/)

##### Optional
- [IPOPT](https://projects.coin-or.org/Ipopt)
- [Qt5](https://www.qt.io/)
- [YARP](https://github.com/robotology/yarp)
- [ICUB](https://github.com/robotology/icub-main)

##### Deprecated
- [Kinematics and Dynamics Library](https://github.com/orocos/orocos_kinematics_dynamics)
- [urdfdom](https://github.com/ros/urdfdom)
- [irrlicht](http://irrlicht.sourceforge.net/)

#### Install dependencies

If you need to install also `YARP` and `ICUB`, it is recommended that you use the [`robotology-superbuild`](https://github.com/robotology/robotology-superbuild), but if you want to install them manually please
check the documentation at [ICub Software Installation](http://wiki.icub.org/wiki/ICub_Software_Installation). The rest of the dependencies can be installed using standard operating system package managers.

##### Windows

There are two ways of installing the required and optional dependencies of iDynTree in Windows: with binary installers or using  [`vcpkg`](https://github.com/Microsoft/vcpkg).

###### Installers
There are several installers that provide the binary dependencies of iDynTree on Windows.

In particular you can run the [binary installer of YARP](http://www.yarp.it/download.html#download_windows) to install Eigen3, Qt5 and YARP itself, the [binary installer of ICUB software](http://wiki.icub.org/wiki/Windows:_installation_from_sources#Getting_iCub.27s_dependenceis) to install Ipopt and ICUB, and the [robotology-additional-dependencies installer](https://github.com/robotology-playground/robotology-additional-dependencies) to install Libxml2 .
**Important: make sure that you are installing the 64-bit installers, if you want to compile the robotology-superbuild using the the 64-bit compiler!**
These installers will set automatically all the enviroment variables necessary to make sure that these libraries are found by CMake, and they will modify the PATH enviroment variable to make sure that the libraries can be used when launching the programs that use them.

###### vcpkg
If you use [`vcpkg`](https://github.com/Microsoft/vcpkg), you can install all the required and optional dependencies of iDynTree using the following command:
~~~
 ./vcpkg install --triplet x64-windows eigen3 qt5 libxml2
~~~
Use the `x86-windows` triplet only if you want to compile iDynTree using the 32-bit compiler.
The default way to use the libraries provided by vcpkg in CMake is to use the [vcpkg CMake toolchain](https://github.com/Microsoft/vcpkg/blob/master/docs/users/integration.md#cmake-toolchain-file-recommended-for-open-source-cmake-projects).
If you prefer not to use the vcpkg toolchain, to use the libraries installed by vcpkg, it is necessary to use the `CMAKE_PREFIX_PATH`,  `CMAKE_PROGRAM_PATH` and `PATH` environment variables. In particular, if your vcpkg is installed in `${VCPKG_ROOT}`,
you need to add `${VCPKG_ROOT}/installed/x64-windows` and `${VCPKG_ROOT}/installed/x64-windows/debug` to `CMAKE_PREFIX_PATH`,
`${VCPKG_ROOT}/installed/x64-windows/tools` to `CMAKE_PROGRAM_PATH` and `${VCPKG_ROOT}/installed/x64-windows/bin` and `${VCPKG_ROOT}/installed/x64-windows/debug/bin` to `PATH`.


##### macOS
You can install most of the required and optional dependencies of iDynTree using [homebrew](https://brew.sh/) with the following command:
~~~
brew install eigen qt5 dartsim/dart/ipopt
~~~


#### Debian/Ubuntu
You can install most of the required and optional dependencies of iDynTree using the following command:
~~~
sudo apt-get install libeigen3-dev libxml2-dev coinor-libipopt-dev qtbase5-dev qtdeclarative5-dev qtmultimedia5-dev qml-module-qtquick2 qml-module-qtquick-window2 qml-module-qtmultimedia qml-module-qtquick-dialogs qml-module-qtquick-controls qml-module-qt-labs-folderlistmodel qml-module-qt-labs-settings
~~~

### Build
Once you installed the necessary dependencies, the iDynTree library can be compiled as any CMake based project.

With `make` facilities:
```bash
$ git clone https://github.com/robotology/idyntree
$ cd idyntree
$ mkdir build && cd build
$ cmake ..
$ make
$ [sudo] make install
```

With IDE build tool facilities, such as Visual Studio or Xcode
```bash
$ git clone https://github.com/robotology/idyntree
$ cd idyntree
$ mkdir build && cd build
$ cmake ..
$ cmake --build . --target ALL_BUILD --config Release
$ cmake --build . --target INSTALL --config Release
```

If you need more help on how to build CMake-based projects, please check [CGold's First step](https://cgold.readthedocs.io/en/latest/first-step.html) section.

In the rest of the documentation, `<prefix>` will indicate the installation prefix in which you installed iDynTree, i.e. the value that you passed as [`CMAKE_INSTALL_PREFIX`](https://cmake.org/cmake/help/latest/variable/CMAKE_INSTALL_PREFIX.html) during the CMake configuration.


### Link
Once the library is installed, you can link it using `CMake` with as little effort as writing the following line of code in your project's `CMakeLists.txt`:
```cmake
...
find_package(iDynTree 0.MINOR.PATCH REQUIRED)
...
target_link_libraries(<target> PRIVATE ${iDynTree_LIBRARIES})
...
```

Note that unless you did not use the default value of `CMAKE_INSTALL_PREFIX`, the `<prefix>` in which you installed iDynTree will need to be appended to the `CMAKE_PREFIX_PATH` enviromental
variable to ensure that `find_package` can find your iDynTree installation.

See [CMake's reference documentation](https://cmake.org/cmake/help/latest/) if you need more info on the [`find_package`](https://cmake.org/cmake/help/latest/command/find_package.html) or [`target_link_libraries`](https://cmake.org/cmake/help/latest/command/target_link_libraries.html) CMake commands.

### Bindings
To compile bindings to iDynTree in several scriping languages, you should enable them using the `IDYNTREE_USES_PYTHON`, `IDYNTREE_USES_LUA`, `IDYNTREE_USES_MATLAB`, `IDYNTREE_USES_OCTAVE` CMake options.

Several examples for using the bindings are available in https://github.com/robotology-playground/idyntree/blob/master/doc/geometric_classes.md .

Then, properly accessing bindings to iDynTree can require some additional steps.

#### Python
You should add to the `PYTHONPATH` enviromental variable the install path of the `iDynTree.py` file.
~~~
export PYTHONPATH=$PYTHONPATH:<prefix>/lib/python2.7/dist-packages/
~~~

#### Matlab
You should add to Matlab path the `<prefix>/mex` directory.
You can modify the relative location for Matlab bindings files in the installation prefix using the `IDYNTREE_INSTALL_MATLAB_LIBDIR` and `IDYNTREE_INSTALL_MATLAB_MFILESDIR` CMake options.

#### Octave
You should add to Octave path the `<prefix>/octave` directory.
You can modify the relative location for Matlab bindings files in the installation prefix using the`IDYNTREE_INSTALL_OCTAVE_LIBDIR` and `IDYNTREE_INSTALL_OCTAVE_MFILESDIR` CMake options.


##### Matlab/Octave bindings modifications
All the other bindings (Python,Lua, ...) are generated by SWIG and compiled on the fly by the user,
by enabling the `IDYNTREE_USES_<LANGUAGE>` option. The Matlab and Octave bindings are an exception because they
rely on an experimental version of Swig, developed for providing Matlab bindings for the [casadi](https://github.com/casadi/casadi/wiki) project. For this reason, usually the Matlab bindigs
are not generated by the users, but by iDynTree developers that have a special experimental Swig
version installed. The bindings code is then committed to the repository, and the `IDYNTREE_USES_MATLAB`
option simply enables *compilation* of the bindings. If you want to regenerate the Matlab bindings,
for example because you modified some iDynTree classes, you can install the experimental
version of Swig with Matlab support from https://github.com/robotology-dependencies/swig/ (branch `matlab`) and then enable Matlab bindings generation with the `IDYNTREE_GENERATE_MATLAB` options.
For more info on how to modify the matlab bindings, see https://github.com/robotology/idyntree/blob/master/doc/dev/faqs.md#how-to-add-wrap-a-new-class-or-function-with-swig .


## Tutorials
| Topic  | C++ | Matlab | Python |
|:------:|:---:|:------:|:------:|
| Use of the [ExtWrenchesAndJointTorquesEstimator class](http://robotology.gitlab.io/docs/idyntree/master/classiDynTree_1_1ExtWrenchesAndJointTorquesEstimator.html) for computing offset for FT sensors | NA | [examples/matlab/SixAxisFTOffsetEstimation/SixAxisFTOffsetEstimation.m](examples/matlab/SixAxisFTOffsetEstimation/SixAxisFTOffsetEstimation.m) | NA |
| How to get the axis of a revolute joint expressed in a arbitary frame using the [KinDynComputations class](http://robotology.gitlab.io/docs/idyntree/master/classiDynTree_1_1KinDynComputations.html) | NA | [ examples/matlab/GetJointAxesInWorldFrame.m](examples/matlab/GetJointAxesInWorldFrame.m) | NA |

Are you interested in a tutorial on a specific feature or algorithm? Just [request it on an enhancement issue](https://github.com/robotology/idyntree/issues/new).

## Documentation
The documentation for the complete API of iDynTree is automatically extracted from the C++ code using [Doxygen](http://www.doxygen.org),
and is available at the URL : [http://robotology.gitlab.io/docs/idyntree/master/](http://robotology.gitlab.io/docs/idyntree/master/).
The documentation generated from the `devel` branch is available at the URL : [http://robotology.gitlab.io/docs/idyntree/devel/](http://robotology.gitlab.io/docs/idyntree/devel/).

## Announcements 
Announcements on new releases, API changes or other news are done on [`robotology/QA` GitHub repository](https://github.com/robotology/QA). You can watch that repository to get all the iDynTree-related announcements, that will always tagged with the `announcement` tag.

## Developer Documentation
If you want to contribute to iDynTree development, please check the [Developer's FAQ](https://github.com/robotology/idyntree/blob/master/doc/dev/faqs.md).

## Reference paper
A paper describing some of the algorithms implemented in iDynTree and their use in a real world scenario can be downloaded [here](http://journal.frontiersin.org/article/10.3389/frobt.2015.00006/abstract) .
If you're going to use this library for your work, please quote it within any resulting publication:
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

## Acknowledgments
The initial development of iDynTree was supported by the FP7 EU projects [CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics)](http://www.codyco.eu/)  and [Koroibot (No. 611909 ICT- 2013.2.1 Cognitive Systems and Robotics)](http://koroibot.eu/).

The development is now supported by the [Dynamic Interaction Control research line](https://www.iit.it/research/lines/dynamic-interaction-control) at the [Italian Institute of Technology](https://www.iit.it/).

## License 
iDynTree is licensed under either the GNU Lesser General Public License v3.0 : 

https://www.gnu.org/licenses/lgpl-3.0.html

or the GNU Lesser General Public License v2.1 :

https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html

at your option.
