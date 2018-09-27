# Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)

macro(add_integration_test testname)
    set(testsrc ${testname}IntegrationTest.cpp)
    set(testbinary ${testname}IntegrationTest)
    set(testtarget   IntegrationTest${testname})
    add_executable(${testbinary} ${testsrc})
    target_include_directories(${testbinary} PRIVATE ${IDYNTREE_TREE_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
    target_link_libraries(${testbinary} idyntree-core idyntree-model idyntree-modelio-urdf idyntree-high-level idyntree-sensors idyntree-estimation)
    add_test(NAME ${testtarget} COMMAND ${testbinary})

    if(IDYNTREE_RUN_VALGRIND_TESTS)
        add_test(NAME memcheck_${testtarget} COMMAND ${MEMCHECK_COMMAND_COMPLETE} $<TARGET_FILE:${testbinary}>)
    endif()
endmacro()

macro(add_integration_test_no_valgrind testname)
    set(testsrc ${testname}IntegrationTest.cpp)
    set(testbinary ${testname}IntegrationTest)
    set(testtarget   IntegrationTest${testname})
    add_executable(${testbinary} ${testsrc})
    target_include_directories(${testbinary} PRIVATE ${IDYNTREE_TREE_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
    target_link_libraries(${testbinary} idyntree-core idyntree-model idyntree-modelio-urdf idyntree-high-level idyntree-sensors idyntree-estimation)
    add_test(NAME ${testtarget} COMMAND ${testbinary})
endmacro()

macro(add_integration_exe testname)
    set(testsrc ${testname}IntegrationTest.cpp)
    set(testbinary ${testname}IntegrationTest)
    set(testtarget   IntegrationTest${testname})
    add_executable(${testbinary} ${testsrc})
    target_include_directories(${testbinary} PRIVATE ${EIGEN3_INCLUDE_DIR} ${IDYNTREE_TREE_INCLUDE_DIRS})
    target_link_libraries(${testbinary} idyntree-core idyntree-model idyntree-modelio-urdf idyntree-high-level)
endmacro()

add_integration_test(Dynamics)
add_integration_test(DenavitHartenberg)

# See issue https://github.com/robotology/idyntree/issues/367
add_integration_test_no_valgrind(iCubTorqueEstimation)

# Until we fix it, add DynamicsLinearization test but don't execute it
add_integration_exe(DynamicsLinearization)

# Some tests
if (IDYNTREE_USES_ASSIMP)
  # Until we fixed the mesh loading ( https://github.com/robotology/idyntree/issues/291 )
  # we just compiled this test without executing it
  add_integration_exe(InertialParametersHelpers)
endif ()
