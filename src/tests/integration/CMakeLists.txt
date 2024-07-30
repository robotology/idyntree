# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


macro(add_integration_test testname)
    set(testsrc ${testname}IntegrationTest.cpp)
    set(testbinary ${testname}IntegrationTest)
    set(testtarget   IntegrationTest${testname})
    add_executable(${testbinary} ${testsrc})
    target_link_libraries(${testbinary} PRIVATE idyntree-core idyntree-model idyntree-modelio
                                                idyntree-high-level idyntree-estimation
                                                idyntree-solid-shapes idyntree-testmodels Eigen3::Eigen)
    # Setting explicitly the WORKING_DIRECTORY is necessary to make sure that meshes are correctly loaded,
    # as a workaround for https://github.com/robotology/idyntree/issues/291
    add_test(NAME ${testtarget} COMMAND ${testbinary} WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/src/tests/data)

    if(IDYNTREE_RUN_VALGRIND_TESTS)
        add_test(NAME memcheck_${testtarget} COMMAND ${MEMCHECK_COMMAND_COMPLETE} $<TARGET_FILE:${testbinary}> WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/src/tests/data)
    endif()
endmacro()

macro(add_integration_test_no_valgrind testname)
    set(testsrc ${testname}IntegrationTest.cpp)
    set(testbinary ${testname}IntegrationTest)
    set(testtarget   IntegrationTest${testname})
    add_executable(${testbinary} ${testsrc})
    target_link_libraries(${testbinary} PRIVATE idyntree-core idyntree-model idyntree-modelio
                                                idyntree-high-level idyntree-estimation
                                                idyntree-solid-shapes idyntree-testmodels Eigen3::Eigen)
    add_test(NAME ${testtarget} COMMAND ${testbinary})
endmacro()

macro(add_integration_exe testname)
    set(testsrc ${testname}IntegrationTest.cpp)
    set(testbinary ${testname}IntegrationTest)
    set(testtarget   IntegrationTest${testname})
    add_executable(${testbinary} ${testsrc})
    target_link_libraries(${testbinary} PRIVATE idyntree-core idyntree-model idyntree-modelio
                                                idyntree-high-level idyntree-testmodels Eigen3::Eigen)
endmacro()

add_integration_test(Dynamics)
add_integration_test(DenavitHartenberg)
add_integration_test(ReducedModelWithFT)
add_integration_test(ModelTransformers)
add_integration_test(iCubTorqueEstimation)

# Until we fix it, add DynamicsLinearization test but don't execute it
add_integration_exe(DynamicsLinearization)

if(IDYNTREE_USES_ASSIMP)
  add_integration_test(InertialParametersSolidShapesHelpers)
endif()
