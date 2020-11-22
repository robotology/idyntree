# Copyright (C) 2013-2018 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

# include test data before everything,
# to be sure that is available to test
# of all components
if(IDYNTREE_COMPILE_TESTS)
    add_subdirectory(tests/data)
endif()

add_subdirectory(core)
add_subdirectory(model)
add_subdirectory(sensors)
add_subdirectory(model_io)
add_subdirectory(estimation)
add_subdirectory(solid-shapes)
add_subdirectory(high-level)
add_subdirectory(inverse-kinematics)

if (IDYNTREE_USES_IPOPT)
  set(IDYNTREE_BUILD_IK TRUE)
endif()

if (IDYNTREE_COMPILES_OPTIMALCONTROL)
  add_subdirectory(optimalcontrol)
endif()

if(IDYNTREE_USES_YARP)
    add_subdirectory(yarp)
endif()

if(IDYNTREE_USES_ICUB_MAIN)
    add_subdirectory(icub)
endif()

add_subdirectory(visualization)

# compile simple command line tools
if(IDYNTREE_COMPILES_TOOLS)
    add_subdirectory(tools)
endif()

if(IDYNTREE_COMPILE_TESTS)
    add_subdirectory(tests)
endif()
