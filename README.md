iDynTree [![License: LGPL v3](https://img.shields.io/badge/License-LGPL%20v3-blue.svg)](https://www.gnu.org/licenses/lgpl-3.0) [![License: LGPL v2](https://img.shields.io/badge/License-LGPL%20v2-blue.svg)](https://www.gnu.org/licenses/lgpl-2.1)  [![ZenHub](https://img.shields.io/badge/Shipping_faster_with-ZenHub-435198.svg)](https://zenhub.com)
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
iDynTree is mainly developed and mantained by the [iCub Tech facility](https://www.iit.it/research/facilities/icub-tech) and [Dynamic Interaction Control research line](https://www.iit.it/research/lines/dynamic-interaction-control) at the [Italian Institute of Technology](https://www.iit.it/), as part of the [iCub project](http://www.icub.org/) .

For this reason it is usually installed through the [robotology-superbuild](https://github.com/robotology/robotology-superbuild), an easy way to download, compile and install the robotology software on multiple operating systems, using the [CMake](www.cmake.org) build system and its extension [YCM](http://robotology.github.io/ycm). To get iDynTree when using the `robotology-superbuild`, please enable the `ROBOTOLOGY_ENABLE_DYNAMICS` CMake option of the superbuild.

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
- [Assimp](http://www.assimp.org/)
- [IPOPT](https://projects.coin-or.org/Ipopt)
- [Qt5](https://www.qt.io/)
- [YARP](https://github.com/robotology/yarp)
- [ICUB](https://github.com/robotology/icub-main)
- [irrlicht](http://irrlicht.sourceforge.net/)

##### Optional for the optimal control part
- [ALGLIB](https://github.com/S-Dafarra/alglib-cmake)
- [osqp-eigen](https://github.com/robotology/osqp-eigen)
- [WORHP](https://worhp.de/)

#### Install dependencies

If you need to install also `YARP` and `ICUB`, it is recommended that you install iDynTree via the [`robotology-superbuild`](https://github.com/robotology/robotology-superbuild). If instead you are not interested in the `YARP` and `ICUB` integration, you can easily install the rest of the dependencies using
standard package managers.

##### Windows

On Windows we recommend to use  [`vcpkg`](https://github.com/Microsoft/vcpkg) C++ package manager to install iDynTree dependencies.

###### vcpkg
If you use [`vcpkg`](https://github.com/Microsoft/vcpkg), you can install all the required and optional dependencies of iDynTree using the following command:
~~~
 ./vcpkg install --triplet x64-windows assimp eigen3 qt5 libxml2 irrlicht
~~~
The default way to use the libraries provided by vcpkg in CMake is to use the [vcpkg CMake toolchain](https://github.com/Microsoft/vcpkg/blob/master/docs/users/integration.md#cmake-toolchain-file-recommended-for-open-source-cmake-projects).

If you want also to install the `ipopt` library, we recommend to use the port `ipopt-binary` available at https://github.com/robotology/robotology-vcpkg-ports .

##### macOS
You can install most of the required and optional dependencies of iDynTree using [homebrew](https://brew.sh/) with the following command:
~~~
brew install assimp eigen qt5 ipopt
~~~

#### Debian/Ubuntu
You can install most of the required and optional dependencies of iDynTree using the following command:
~~~
sudo apt-get install libeigen3-dev libxml2-dev coinor-libipopt-dev qtbase5-dev qtdeclarative5-dev qtmultimedia5-dev qml-module-qtquick2 qml-module-qtquick-window2 qml-module-qtmultimedia qml-module-qtquick-dialogs qml-module-qtquick-controls qml-module-qt-labs-folderlistmodel qml-module-qt-labs-settings
~~~

### Build
Once you installed the necessary dependencies, the iDynTree library can be compiled as any CMake based project. In the following instructions, we indicate with `<additional_platform_specific_options>` where
you should add the platform specific options, as the use of `-DCMAKE_TOOLCHAIN_FILE=[path to vcpkg]/scripts/buildsystems/vcpkg.cmake` if you are using [vcpkg](https://github.com/Microsoft/vcpkg).

With `make` facilities:
```bash
$ git clone https://github.com/robotology/idyntree
$ cd idyntree
$ mkdir build && cd build
$ cmake <additional_platform_specific_options> ..
$ make
$ [sudo] make install
```

With IDE build tool facilities, such as Visual Studio or Xcode
```bash
$ git clone https://github.com/robotology/idyntree
$ cd idyntree
$ mkdir build && cd build
$ cmake <additional_platform_specific_options> ..
$ cmake --build . --target ALL_BUILD --config Release
$ cmake --build . --target INSTALL --config Release
```


If you need more help on how to build CMake-based projects, please check [CGold's First step](https://cgold.readthedocs.io/en/latest/first-step.html) section.

In the rest of the documentation, `<prefix>` will indicate the installation prefix in which you installed iDynTree, i.e. the value that you passed as [`CMAKE_INSTALL_PREFIX`](https://cmake.org/cmake/help/latest/variable/CMAKE_INSTALL_PREFIX.html) during the CMake configuration.


### Link
Once the library is installed, you can link it using `CMake` with as little effort as writing the following line of code in your project's `CMakeLists.txt`:
```cmake
...
find_package(iDynTree REQUIRED)
...
target_link_libraries(<target> PRIVATE ${iDynTree_LIBRARIES})
...
```

Note that unless you did not use the default value of `CMAKE_INSTALL_PREFIX`, the `<prefix>` in which you installed iDynTree will need to be appended to the `CMAKE_PREFIX_PATH` enviromental
variable to ensure that `find_package` can find your iDynTree installation.

See [CMake's reference documentation](https://cmake.org/cmake/help/latest/) if you need more info on the [`find_package`](https://cmake.org/cmake/help/latest/command/find_package.html) or [`target_link_libraries`](https://cmake.org/cmake/help/latest/command/target_link_libraries.html) CMake commands.

### Bindings
To compile bindings to iDynTree in several scriping languages, you should enable them using the `IDYNTREE_USES_PYTHON`, `IDYNTREE_USES_LUA`, `IDYNTREE_USES_MATLAB`, `IDYNTREE_USES_OCTAVE` CMake options.

Then, properly accessing bindings to iDynTree can require some additional steps.

#### Python
You should add to the `PYTHONPATH` enviromental variable the install path of the `iDynTree.py` file.
~~~
export PYTHONPATH=$PYTHONPATH:<prefix>/lib/python<majorPythonVersion>.<minorPythonVersion>./dist-packages/
~~~

#### Python (pybind11)
To compile the python bindings based on [pybind11](https://github.com/pybind/pybind11) set to `TRUE` the `IDYNTREE_USES_PYTHON_PYBIND11` option.

**NOTE**: the generated bindings are not compatible with the bindings generated by SWIG. Do not expect your Python code to use either of the two without modifications.

##### Use
Modify your `PYTHONPATH` environment variable to point to the bindings installation directory:

```sh
export PYTHONPATH=${PYTHONPATH}:<prefix>/<python_package_path>
```

where `<prefix>` corresponds to the value specified in `CMAKE_INSTALL_PREFIX` and `<python_package_path>` is the Python installation prefix, as returned by

```python
disutils.sysconfig.get_python_lib(1,0,prefix='')
```

for example: `lib/python3.8/site-packages`.

Finally to use the bindings in your Python code, simply import the package:

```python
import idyntree.pybind as iDynTree

```

#### MATLAB
You should add to Matlab path the `<prefix>/mex` directory.
You can modify the relative location for Matlab bindings files in the installation prefix using the `IDYNTREE_INSTALL_MATLAB_LIBDIR` and `IDYNTREE_INSTALL_MATLAB_MFILESDIR` CMake options.

#### Octave
You should add to Octave path the `<prefix>/octave` directory.
You can modify the relative location for Matlab bindings files in the installation prefix using the`IDYNTREE_INSTALL_OCTAVE_LIBDIR` and `IDYNTREE_INSTALL_OCTAVE_MFILESDIR` CMake options.


##### MATLAB/Octave bindings modifications
All the other bindings (Python,Lua, ...) are generated by SWIG and compiled on the fly by the user,
by enabling the `IDYNTREE_USES_<LANGUAGE>` option. The Matlab and Octave bindings are an exception because they
rely on an experimental version of Swig, developed for providing Matlab bindings for the [casadi](https://github.com/casadi/casadi/wiki) project. For this reason, usually the Matlab bindigs
are not generated by the users, but by iDynTree developers that have a special experimental Swig
version installed. The bindings code is then committed to the repository, and the `IDYNTREE_USES_MATLAB`
option simply enables *compilation* of the bindings. If you want to regenerate the Matlab bindings,
for example because you modified some iDynTree classes, you can install the experimental
version of Swig with Matlab support from https://github.com/robotology-dependencies/swig/ (branch `matlab`) and then enable Matlab bindings generation with the `IDYNTREE_GENERATE_MATLAB` options.
For more info on how to modify the matlab bindings, see https://github.com/robotology/idyntree/blob/master/doc/dev/faqs.md#how-to-add-wrap-a-new-class-or-function-with-swig .

##### MATLAB/Octave high level wrappers
They are a collection of Matlab/Octave functions that wraps the functionalities of (mainly) the iDyntree class `KinDynComputations` into functions with a typical Matlab/Octave interface. The purpose of the high-level wrappers is to provide a simpler and easy-to-use interface for Matlab/Octave users who want to use iDyntree inside Matlab/Octave, also helping in designing code which is less error-prone and easier to debug (e.g. in case the interface of an iDyntree function will change in the future). More details and a complete list of the wrappers can be found in the [wrappers README](/bindings/matlab/+iDynTreeWrappers/README.md).

**Usage**: the wrappers package is installed together with the iDyntree bindings when compiling iDyntree with option `IDYNTREE_USES_MATLAB` or `IDYNTREE_USES_OCTAVE` set to `ON`. The functions can be called from Matlab/Octave using the namespace `iDynTreeWrappers`, i.e. `iDynTreeWrappers.name_of_the_corresponding_iDynTree_method`.

## Tutorials
| Topic  | Location | Language  |
|:------:|:--------:|:---------:|
| Basic usage of the [KinDynComputations class](https://robotology.github.io/idyntree/master/classiDynTree_1_1KinDynComputations.html) together with the [[Eigen](http://eigen.tuxfamily.org) C++ Matrix library. | [examples/cxx/KinDynComputationsWithEigen/main.cpp](examples/cxx/KinDynComputationsWithEigen/main.cpp) | C++ |
| How to use the [InverseKinematics class](https://robotology.github.io/docs/idyntree/master/classiDynTree_1_1InverseKinematics.html) for the IK of an industrial fixed-base manipulator. | [examples/cxx/InverseKinematics/README.md](examples/cxx/InverseKinematics/README.md) | C++ |
| Use of the [ExtWrenchesAndJointTorquesEstimator class](https://robotology.github.io/idyntree/master/classiDynTree_1_1ExtWrenchesAndJointTorquesEstimator.html) for computing offset for FT sensors |  [examples/matlab/SixAxisFTOffsetEstimation/SixAxisFTOffsetEstimation.m](examples/matlab/SixAxisFTOffsetEstimation/SixAxisFTOffsetEstimation.m) | MATLAB |
| How to get the axis of a revolute joint expressed in a arbitary frame using the [KinDynComputations class](https://robotology.github.io/idyntree/master/classiDynTree_1_1KinDynComputations.html) | [examples/matlab/SensorsListParsing/SensorsListParsing.m](examples/matlab/SensorsListParsing/SensorsListParsing.m) | MATLAB |
| How to read the Six Axis Force Torque sensors information contained in a URDF model. | [examples/matlab/GetJointAxesInWorldFrame.m](examples/matlab/GetJointAxesInWorldFrame.m) | MATLAB |
| Usage of the MATLAB-native visualizer using the [MATLAB high-level wrappers](bindings/matlab/+iDynTreeWrappers/README.md). | [examples/matlab/iDynTreeWrappers/visualizeRobot.m](examples/matlab/iDynTreeWrappers/visualizeRobot.m) | MATLAB |
| Basic usage of the [KinDynComputations class](https://robotology.github.io/idyntree/master/classiDynTree_1_1KinDynComputations.html). | [examples/python/KinDynComputationsTutorial.py](examples/python/KinDynComputationsTutorial.py) | Python |

Are you interested in a tutorial on a specific feature or algorithm? Just [request it on an enhancement issue](https://github.com/robotology/idyntree/issues/new).

## Documentation
The documentation for the complete API of iDynTree is automatically extracted from the C++ code using [Doxygen](http://www.doxygen.org),
and is available at the URL : [https://robotology.github.io/idyntree/master/](https://robotology.github.io/idyntree/master/).
The documentation generated from the `devel` branch is available at the URL : [https://robotology.github.io/idyntree/devel/](https://robotology.github.io/idyntree/devel/).

## Announcements
Announcements on new releases, API changes or other news are done on [`robotology/QA` GitHub repository](https://github.com/robotology/QA). You can watch that repository to get all the iDynTree-related announcements, that will always tagged with the `announcement` tag.

## Developer Documentation
If you want to contribute to iDynTree development, please check the [Developer's FAQ](doc/dev/faqs.md).

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
