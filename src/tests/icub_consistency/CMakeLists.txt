# Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.



macro(add_icub_consistency_test testName)
    set(testsrc ${testName}ConsistencyTest.cpp)
    set(testbinary ${testName}ConsistencyTest)
    set(testname   ConsistencyTest${testName})
    add_executable(${testbinary} ${testsrc})
    target_link_libraries(${testbinary} PRIVATE idyntree-yarp idyntree-high-level idyntree-icub idyntree-core
                                                idyntree-model idyntree-estimation idyntree-testmodels Eigen3::Eigen)
    add_test(NAME ${testname} COMMAND ${testbinary})

    if(IDYNTREE_RUN_VALGRIND_TESTS)
        add_test(NAME memcheck_${testname} COMMAND ${MEMCHECK_COMMAND_COMPLETE} $<TARGET_FILE:${testbinary}>)

    endif()
endmacro()

add_icub_consistency_test(iKin)
if (IDYNTREE_USES_KDL)
    add_icub_consistency_test_kdl(iCubExternalWrenchesEstimation)
endif ()
