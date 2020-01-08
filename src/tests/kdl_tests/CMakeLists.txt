# Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

# Legacy tests, remove deprecated warnings when compiling
idyntree_disable_deprecation_warnings()

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR} ${orocos_kdl_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})

macro(add_all_components_test testName)
    set(testsrc ${testName}_test.cpp testhumanoid.cpp testdoublejoint.cpp testkukalwr.cpp testsinglejoint.cpp testsinglelink.cpp)
    set(testbinary ${testName}_test)
    set(testname   test_${testName})
    add_executable(${testbinary} ${testsrc})
    target_include_directories(${testbinary} PRIVATE ${EIGEN3_INCLUDE_DIR})
    target_link_libraries(${testbinary} idyntree-core idyntree-high-level-kdl idyntree-yarp-kdl ${YARP_LIBRARIES} ${orocos_kdl_LIBRARIES} idyntree-testmodels)
    add_test(NAME ${testname} COMMAND ${testbinary})

    if(IDYNTREE_RUN_VALGRIND_TESTS)
        add_test(NAME memcheck_${testname} COMMAND ${MEMCHECK_COMMAND_COMPLETE} $<TARGET_FILE:${testbinary}>)

    endif()
endmacro()

if(IDYNTREE_USES_YARP)
  add_all_components_test(inverse_inertia)
  add_all_components_test(regressor_utils)
  add_all_components_test(subtree_regressor)
  add_all_components_test(inversedynamics_regressor)
  add_all_components_test(inversedynamics_crba)
  add_all_components_test(comjacobian)
  add_all_components_test(inversedynamics_different_loops)
  add_all_components_test(DynamicsComputationsConsistency)
endif()

if(IDYNTREE_USES_ICUB_MAIN)
  # Commented because at the moment it takes a lot of time
  # add_all_components_test(urdf2dh)
  # target_link_libraries(urdf2dh_test iKin idyntree-kdl idyntree-yarp-kdl idyntree-regressors idyntree-modelio-urdf-kdl idyntree-modelio-ikin)
  add_executable(urdf2dh_standalone_test urdf2dh_test.cpp)
  target_link_libraries(urdf2dh_standalone_test iKin idyntree-kdl idyntree-yarp-kdl idyntree-regressors idyntree-modelio-urdf-kdl idyntree-modelio-ikin-kdl idyntree-testmodels)
# \todo reenable this test
  #add_test(NAME test_urdf2dh COMMAND urdf2dh_test urdf_icub_test.urdf)
endif()

FIND_PACKAGE(Boost)
include_directories(${Boost_INCLUDE_DIR})
add_executable(urdf_forward_kinematics urdf_forward_kinematics.cpp)
target_link_libraries(urdf_forward_kinematics idyntree-core idyntree-kdl idyntree-modelio-urdf-kdl ${orocos_kdl_LIBRARIES})

#add_test(NAME test_urdf_forward_kinematics  COMMAND urdf_forward_kinematics black_icub.urdf root_link r_hip_3)
