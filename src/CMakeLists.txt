# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

# include test data before everything,
# to be sure that is available to test
# of all components
if(BUILD_TESTING)
    add_subdirectory(tests/data)
endif()

add_subdirectory(core)
add_subdirectory(model)
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

add_subdirectory(yarp)
add_subdirectory(icub)

add_subdirectory(visualization)

# compile simple command line tools
if(IDYNTREE_COMPILES_TOOLS)
    add_subdirectory(tools)
endif()

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
