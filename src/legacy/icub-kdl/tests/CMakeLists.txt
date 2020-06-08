include_directories(${CMAKE_SOURCE_DIR}/include
                          ${EIGEN3_INCLUDE_DIR}
                          ${orocos_kdl_INCLUDE_DIRS}
                          ${iDynTree_INCLUDE_DIRS}
                          ${YARP_INCLUDE_DIRS}
                          ${ICUB_INCLUDE_DIRS})

macro(add_icub_test testName)
    set(testsrc ${testName}Test.cpp)
    set(testbinary ${testName}Test)
    set(testname   test_${testName})
    add_executable(${testbinary} ${testsrc})
    target_include_directories(${testbinary} PRIVATE ${EIGEN3_INCLUDE_DIR})
    target_link_libraries(${testbinary} idyntree-core idyntree-high-level idyntree-yarp-kdl idyntree-icub-kdl idyntree-kdl idyntree-modelio-urdf-kdl idyntree-modelio-urdf YARP::YARP_os YARP::YARP_math ${orocos_kdl_LIBRARIES} idyntree-testmodels)
    add_test(NAME ${testname} COMMAND ${testbinary})

    if(IDYNTREE_RUN_VALGRIND_TESTS)
        add_test(NAME memcheck_${testname} COMMAND ${MEMCHECK_COMMAND_COMPLETE} $<TARGET_FILE:${testbinary}>)

    endif()
endmacro()

add_icub_test(iCubLeftArmRegressor)
add_icub_test(iDynTreeCOM)
add_icub_test(iDynTreeJacobian)
add_icub_test(iDynTreeRelativeJacobian)
add_icub_test(iDynTreeWholeBodyFTsensorEstimate)
add_icub_test(iDynTreeExportFixedJointsFrames)
add_icub_test(iDynTreeExternalWrenchesEstimation)
add_icub_test(AdjointMatricesConsistency)
