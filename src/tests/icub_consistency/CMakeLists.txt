# Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)
include_directories(${IDYNTREE_TREE_INCLUDE_DIRS})

macro(add_icub_consistency_test_kdl testName)
    set(testsrc ${testName}ConsistencyTest.cpp)
    set(testbinary ${testName}ConsistencyTest)
    set(testname   ConsistencyTest${testName})
    add_executable(${testbinary} ${testsrc})
    target_link_libraries(${testbinary} idyntree-modelio-urdf-kdl idyntree-yarp idyntree-high-level idyntree-modelio-ikin-kdl idyntree-icub idyntree-icub-kdl idyntree-icub idyntree-core idyntree-kdl idyntree-model idyntree-estimation)
    target_include_directories(${testbinary} PRIVATE SYSTEM ${EIGEN3_INCLUDE_DIR})
    add_test(NAME ${testname} COMMAND ${testbinary})

    if(IDYNTREE_RUN_VALGRIND_TESTS)
        add_test(NAME memcheck_${testname} COMMAND ${MEMCHECK_COMMAND_COMPLETE} $<TARGET_FILE:${testbinary}>)

    endif()
endmacro()

macro(add_icub_consistency_test testName)
    set(testsrc ${testName}ConsistencyTest.cpp)
    set(testbinary ${testName}ConsistencyTest)
    set(testname   ConsistencyTest${testName})
    add_executable(${testbinary} ${testsrc})
    target_link_libraries(${testbinary} idyntree-yarp idyntree-high-level idyntree-icub idyntree-icub idyntree-icub idyntree-core idyntree-model idyntree-estimation)
    target_include_directories(${testbinary} PRIVATE SYSTEM ${EIGEN3_INCLUDE_DIR})
    add_test(NAME ${testname} COMMAND ${testbinary})

    if(IDYNTREE_RUN_VALGRIND_TESTS)
        add_test(NAME memcheck_${testname} COMMAND ${MEMCHECK_COMMAND_COMPLETE} $<TARGET_FILE:${testbinary}>)

    endif()
endmacro()

add_icub_consistency_test(iKin)
if (IDYNTREE_USES_KDL)
    add_icub_consistency_test_kdl(iCubExternalWrenchesEstimation)
endif ()