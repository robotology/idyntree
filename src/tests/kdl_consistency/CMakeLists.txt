# Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)

# Legacy tests, remove deprecated warnings when compiling
idyntree_disable_deprecation_warnings()

macro(add_kdl_consistency_test testName)
    set(testsrc ${testName}ConsistencyTest.cpp)
    set(testbinary ${testName}ConsistencyTest)
    set(testname   ConsistencyTest${testName})
    add_executable(${testbinary} ${testsrc})
    target_include_directories(${testbinary} PRIVATE ${IDYNTREE_TREE_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
    target_link_libraries(${testbinary} idyntree-modelio-urdf-kdl idyntree-core idyntree-kdl idyntree-model)
    add_test(NAME ${testname} COMMAND ${testbinary})

    if(IDYNTREE_RUN_VALGRIND_TESTS)
        add_test(NAME memcheck_${testname} COMMAND ${MEMCHECK_COMMAND_COMPLETE} $<TARGET_FILE:${testbinary}>)

    endif()
endmacro()


add_kdl_consistency_test(Geometry)
add_kdl_consistency_test(KinematicsDynamics)