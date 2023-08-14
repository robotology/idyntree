# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


macro(add_estimation_test classname)
    set(testsrc ${classname}UnitTest.cpp)
    set(testbinary ${classname}UnitTest)
    set(testname   UnitTest${classname})
    add_executable(${testbinary} ${testsrc})
    target_link_libraries(${testbinary} PRIVATE idyntree-model idyntree-estimation idyntree-testmodels Eigen3::Eigen)
    add_test(NAME ${testname} COMMAND ${testbinary})

    if(IDYNTREE_RUN_VALGRIND_TESTS)
        add_test(NAME memcheck_${testname} COMMAND ${MEMCHECK_COMMAND_COMPLETE} $<TARGET_FILE:${testbinary}>)

    endif()
endmacro()

add_estimation_test(BerdyHelper)
add_estimation_test(BerdyMAPSolver)
add_estimation_test(ExternalWrenchesEstimation)
add_estimation_test(ExtWrenchesAndJointTorquesEstimator)
add_estimation_test(SimpleLeggedOdometry)
add_estimation_test(AttitudeEstimator)
add_estimation_test(KalmanFilter)
