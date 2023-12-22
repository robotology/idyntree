iDynTree  [![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause) [![ZenHub](https://img.shields.io/badge/Shipping_faster_with-ZenHub-435198.svg)](https://zenhub.com)
===========

iDynTree is a library of robots dynamics algorithms for control, estimation and simulation. It is specifically designed for free-floating robots, but it is possible to use it also  with fixed-base robots.

The **major characteristic features** of iDynTree are:
* It is written in **C++**, with **Python** and **MATLAB** bindings.
* It uses an  **undirected graph data structure** (`iDynTree::Model`) that is used to represent robots, to easily change the **base link** that you are using for your kinematics and dynamics computations without the need to reload your model or change your joint or link serializations. This is done as iDynTree was developed for floating-base robots such as humanoids, in which the most convenient choice of **base link** can change.
* It contains support for **reading and writing URDF files** from a `iDynTree::Model`, making it useful to write tools that modify robot models and saves them back to file. This is done as iDynTree was meant to develop **tools for identification of kinematics and dynamics parameters**.
* It defaults to use the **mixed representation** to represent link quantities (including the velocity and acceleration of the base link), but it can optionally use also **body (left-trivialized)** or **inertial (right-trivialized)** representation if requested. This is done because iDynTree was developed to satisfy the needs of **research in synthesis of floating-base whole-body controllers**. If you are not familiar with the different representation for 6D quantities, check  [Section 6 of "Multibody dynamics notation (version 2)"](
https://pure.tue.nl/ws/portalfiles/portal/139293126/A_Multibody_Dynamics_Notation_Revision_2_.pdf).
* It contains an implementation of the algorithm used in the iCub humanoid robot to **estimate the joint torques without the need of collocated joint torque sensors**, exploting the specific **undirected graph data structure** . This is done as this was one of the originally goal for the implementation of iDynTree. See the class [`iDynTree::ExtWrenchesAndJointTorquesEstimator`](https://github.com/robotology/idyntree/blob/master/src/estimation/include/iDynTree/ExtWrenchesAndJointTorquesEstimator.h) and [Chapter 6 of "Modelling, Estimation and Identification of Humanoid Robots Dynamics"](https://traversaro.github.io/traversaro-phd-thesis/traversaro-phd-thesis.pdf).

To avoid confusion, it is also useful to clarify what **iDynTree is not**:
* It is not the **fastest C++ library** for kinematics and dynamics multibody computations for robotics. It is not slow, but if have an application in which you need the absolute fastest library, check out [Pinocchio](https://github.com/stack-of-tasks/pinocchio).
* It is not a **multibody simulator** library. It provides the building blocks that you could use to build a multibody simulator, but it is not a multibody simulator per se. If you need a simulator library in C++, check out [DART](https://dartsim.github.io/), [Simbody](https://github.com/simbody/simbody), [Drake](https://drake.mit.edu/), [MuJoCo](https://mujoco.org/) or the abstraction layer [`Gazebo Physics`](https://github.com/gazebosim/gz-physics). If you need a simulator implemented in MATLAB/Simulink (built on iDynTree), check [`matlab-whole-body-simulator`](https://github.com/ami-iit/matlab-whole-body-simulator). If you need a simulator that is differentiable and runs on GPU, check [`jaxsim`](https://github.com/ami-iit/jaxsim) or [MuJoCo XLA (mjx)](https://mujoco.readthedocs.io/en/stable/mjx.html).
* It does not provide algorithms in a form in which they can be used in [CasADi](http://casadi.org/), [JAX](https://jax.readthedocs.io/en/latest/) or [PyTorch](https://pytorch.org/). For a Python library with an interface inspired by iDynTree that provides algorithms compatible with these frameworks, check [`adam`](https://github.com/ami-iit/adam) robotics library.


##  Contents
* **[Installation](#installation)**
* **[Library Usage](#library-usage)**
* **[Tools Usage](#tools-usage)**
* **[Reference Documentation](#reference-documentation)**
* **[Announcements](#announcements)**
* **[Developer Documentation](#developer-documentation)**
* **[Reference Paper](#reference-paper)**
* **[Acknowledgments](#acknowledgments)**

## Installation

### conda (recommended) 

You can easily install the C++ and Python library with via [`conda-forge`](https://conda-forge.org) using the following command
~~~
conda install -c conda-forge idyntree
~~~

If you need to install also the MATLAB bindings, you can install them with:
~~~
conda install -c conda-forge -c robotology idyntree-matlab-bindings
~~~

If you are not familiar with conda or conda-forge, you can read an introduction document in [conda-forge overview](https://github.com/robotology/robotology-superbuild/blob/master/doc/conda-forge.md#conda-forge-overview).

### robotology-superbuild (advanced)

If you are installing iDynTree for use as part of [iCub humanoid robot software installation](https://icub-tech-iit.github.io/documentation/sw_installation/), you may want to install iDynTree through the [robotology-superbuild](https://github.com/robotology/robotology-superbuild), an easy way to download, compile and install the robotology software on multiple operating systems, using the [CMake](https://www.cmake.org) build system and its extension [YCM](http://robotology.github.io/ycm). To get iDynTree when using the `robotology-superbuild`, please enable the `ROBOTOLOGY_ENABLE_DYNAMICS` CMake option of the superbuild. If you want to install also iDynTree Python or MATLAB bindings, remember to enable the `ROBOTOLOGY_USES_PYTHON` or `ROBOTOLOGY_USES_MATLAB` options. 

### Build from source (advanced)

If you want to build iDynTree directly from source, you can check the documentation in [`doc/build-from-source.md`](doc/build-from-source.md).

## Library Usage

### Usage in C++
Once the library is installed, you can link it in C++ programs using `CMake` with as little effort as writing the following line of code in your project's `CMakeLists.txt`:
```cmake
find_package(iDynTree REQUIRED)
target_link_libraries(<target> PRIVATE iDynTree::idyntree-high-level iDynTree::idyntree-estimation)
```

See [CMake's reference documentation](https://cmake.org/cmake/help/latest/) if you need more info on the [`find_package`](https://cmake.org/cmake/help/latest/command/find_package.html) or [`target_link_libraries`](https://cmake.org/cmake/help/latest/command/target_link_libraries.html) CMake commands.

### Usage in MATLAB
To make sure that iDynTree is available in MATLAB, try to run some simple code that uses it:
~~~
p = iDynTree.Position()
~~~

If this is not working, make sure that you are launching `matlab` after having activated the conda environment (if you installed iDynTree via conda) or after having sourced por executed the correct setup script (if you installed iDynTree via the robotology-superbuild).

### Tutorials

These tutorials describe how to use specific parts of iDynTree. Are you interested in a tutorial on a specific feature or algorithm that you can't find in this list? Just [request it on an enhancement issue](https://github.com/robotology/idyntree/issues/new).

| Topic  | Location | Language  |
|:------:|:--------:|:---------:|
| Basic usage of the [KinDynComputations class](https://robotology.github.io/idyntree/classiDynTree_1_1KinDynComputations.html) together with the [[Eigen](http://eigen.tuxfamily.org) C++ Matrix library to compute kinematics and dynamics quantities such as forward kinematics, inverse dynamics, mass matrix. | [examples/cxx/KinDynComputationsWithEigen/main.cpp](examples/cxx/KinDynComputationsWithEigen/main.cpp) | C++ |
| How to use the [InverseKinematics class](https://robotology.github.io/idyntree/classiDynTree_1_1InverseKinematics.html) for the IK of an industrial fixed-base manipulator. | [examples/cxx/InverseKinematics/README.md](examples/cxx/InverseKinematics/README.md) | C++ |
| Use of the [ExtWrenchesAndJointTorquesEstimator class](https://robotology.github.io/idyntree/classiDynTree_1_1ExtWrenchesAndJointTorquesEstimator.html) for computing offset for FT sensors |  [examples/matlab/SixAxisFTOffsetEstimation/SixAxisFTOffsetEstimation.m](examples/matlab/SixAxisFTOffsetEstimation/SixAxisFTOffsetEstimation.m) | MATLAB |
| How to get the axis of a revolute joint expressed in a arbitary frame using the [KinDynComputations class](https://robotology.github.io/idyntree/classiDynTree_1_1KinDynComputations.html) | [examples/matlab/SensorsListParsing/SensorsListParsing.m](examples/matlab/SensorsListParsing/SensorsListParsing.m) | MATLAB |
| How to read the Six Axis Force Torque sensors information contained in a URDF model. | [examples/matlab/GetJointAxesInWorldFrame.m](examples/matlab/GetJointAxesInWorldFrame.m) | MATLAB |
| Usage of the MATLAB-native visualizer using the [MATLAB high-level wrappers](bindings/matlab/+iDynTreeWrappers/README.md). | [examples/matlab/iDynTreeWrappers/visualizeRobot.m](examples/matlab/iDynTreeWrappers/visualizeRobot.m) | MATLAB |
| Basic usage of the [KinDynComputations class](https://robotology.github.io/idyntree/classiDynTree_1_1KinDynComputations.html). | [examples/python/KinDynComputationsTutorial.py](examples/python/KinDynComputationsTutorial.py) | Python |
| Basic usage of the [MeshcatVisualizer class](bindings/python/visualize/meshcat_visualizer.py). | [examples/python/MeshcatVisualizerExample.ipynb](examples/python/MeshcatVisualizerExample.ipynb) | Python |

## Tools Usage 

iDynTree also includes some command line tools to use some of the functionality of the library without writing any line of code. The available command line tools are listed in the following, and each tool also includes an online help that is tipically available by passing the `-h` flag.

### `idyntree-model-info`

Tool that reads a model from a file, and print some useful information as specified via the command line.

Example: Print the total mass of a given model
~~~
idyntree-model-info -m <location-of-the-model> --total-mass
~~~

### `idyntree-model-view`

Tool that reads a model from a file and visualize it using the `idyntree-visualizer` library

Example: Visualize a given model
~~~
idyntree-model-view -m <location-of-the-model>
~~~

### `idyntree-model-simplify-shapes`

Tool that reads a model from a file, and returns in output the same model, but with all 
solid shapes of the model (both collision and visual) substituted with a primitive shape 
that approximates in some way the original solid shape. At the moment, the only conversion 
type provided is to approximate each solid shape of the model with its axis aligned bounding box.

Example: Approximate a given model
~~~
idyntree-model-simplify-shapes -m <location-of-the-input-model> -o <desired-location-of-the-output-model>
~~~


## Reference Documentation
The documentation for the complete API of iDynTree is automatically extracted from the C++ code using [Doxygen](http://www.doxygen.org),
and is available at the URL : [https://robotology.github.io/idyntree](https://robotology.github.io/idyntree).

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
The initial development of iDynTree was supported by the FP7 EU projects [CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics)](http://www.codyco.eu/)  and [Koroibot (No. 611909 ICT- 2013.2.1 Cognitive Systems and Robotics)](https://cordis.europa.eu/project/id/611909/).

The development is now supported by the [Artificial Mechanical Intelligence research line](https://ami.iit.it/) at the [Italian Institute of Technology](https://www.iit.it/).

## License

iDynTree is licensed under either the BSD-3-Clause license : https://spdx.org/licenses/BSD-3-Clause.html .
