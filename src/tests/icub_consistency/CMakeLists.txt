# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause




macro(add_icub_consistency_test testName)
    set(testsrc ${testName}ConsistencyTest.cpp)
    set(testbinary ${testName}ConsistencyTest)
    set(testname   ConsistencyTest${testName})
    add_executable(${testbinary} ${testsrc})
    target_link_libraries(${testbinary} PRIVATE idyntree-yarp idyntree-high-level idyntree-icub idyntree-core
                                                idyntree-model idyntree-estimation idyntree-testmodels iKin Eigen3::Eigen)
    add_test(NAME ${testname} COMMAND ${testbinary})

    if(IDYNTREE_RUN_VALGRIND_TESTS)
        add_test(NAME memcheck_${testname} COMMAND ${MEMCHECK_COMMAND_COMPLETE} $<TARGET_FILE:${testbinary}>)

    endif()
endmacro()

add_icub_consistency_test(iKin)
if (IDYNTREE_USES_KDL)
    add_icub_consistency_test_kdl(iCubExternalWrenchesEstimation)
endif ()
