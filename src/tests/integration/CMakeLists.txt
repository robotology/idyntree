# Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

macro(add_integration_test testname)
    set(testsrc ${testname}IntegrationTest.cpp)
    set(testbinary ${testname}IntegrationTest)
    set(testtarget   IntegrationTest${testname})
    add_executable(${testbinary} ${testsrc})
    target_link_libraries(${testbinary} PRIVATE idyntree-core idyntree-model idyntree-modelio-urdf
                                                idyntree-high-level idyntree-sensors idyntree-estimation
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
    target_link_libraries(${testbinary} PRIVATE idyntree-core idyntree-model idyntree-modelio-urdf
                                                idyntree-high-level idyntree-sensors idyntree-estimation
                                                idyntree-solid-shapes idyntree-testmodels Eigen3::Eigen)
    add_test(NAME ${testtarget} COMMAND ${testbinary})
endmacro()

macro(add_integration_exe testname)
    set(testsrc ${testname}IntegrationTest.cpp)
    set(testbinary ${testname}IntegrationTest)
    set(testtarget   IntegrationTest${testname})
    add_executable(${testbinary} ${testsrc})
    target_link_libraries(${testbinary} PRIVATE idyntree-core idyntree-model idyntree-modelio-urdf
                                                idyntree-high-level idyntree-testmodels Eigen3::Eigen)
endmacro()

add_integration_test(Dynamics)
add_integration_test(DenavitHartenberg)

# See issue https://github.com/robotology/idyntree/issues/367
add_integration_test_no_valgrind(iCubTorqueEstimation)

# Until we fix it, add DynamicsLinearization test but don't execute it
add_integration_exe(DynamicsLinearization)

if(IDYNTREE_USES_ASSIMP)
  add_integration_test(InertialParametersSolidShapesHelpers)
endif()
