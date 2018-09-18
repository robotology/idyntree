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
add_subdirectory(high-level)

if (IDYNTREE_USES_IPOPT)
  set(IDYNTREE_BUILD_IK TRUE)
  add_subdirectory(inverse-kinematics)
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

if(IDYNTREE_USES_KDL)
    add_subdirectory(legacy/kdl)
    if(IDYNTREE_USES_YARP)
        add_subdirectory(legacy/yarp-kdl)
        add_subdirectory(legacy/estimation-kdl)
        if(IDYNTREE_USES_ICUB_MAIN)
            add_subdirectory(legacy/icub-kdl)
        endif()
    endif()
endif()

# This should directory should not compiled if IDYNTREE_USES_KDL
# is not enabled, but they are necessary to compile the bindings,
# see https://github.com/robotology/idyntree/issues/227
add_subdirectory(legacy/high-level-kdl)
add_subdirectory(regressors)

add_subdirectory(visualization)

# compile simple command line tools
add_subdirectory(tools)

if(IDYNTREE_COMPILE_TESTS)
    add_subdirectory(tests)
endif()
