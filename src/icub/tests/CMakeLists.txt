

find_package(TinyXML       REQUIRED)
include_directories(${CMAKE_SOURCE_DIR}/include
                          ${EIGEN3_INCLUDE_DIR}
                          ${orocos_kdl_INCLUDE_DIRS}
                          ${iDynTree_INCLUDE_DIRS}
                          ${YARP_INCLUDE_DIRS}
                          ${kdl_codyco_INCLUDE_DIRS}
                          ${ICUB_INCLUDE_DIRS})

add_executable(iDynTreeJacobianTest         iDynTreeJacobianTest.cpp)
add_executable(iDynTreeRelativeJacobianTest iDynTreeRelativeJacobianTest.cpp)
add_executable(iDynTreeWholeBodyFTsensorEstimateTest iDynTreeWholeBodyFTsensorEstimateTest.cpp)
add_executable(iDynTreeExportFixedJointsFrames iDynTreeExportFixedJointsFrames.cpp)
add_executable(iDynTreeExternalWrenchesEstimationTest iDynTreeExternalWrenchesEstimationTest.cpp)
add_executable(iDynTreeiCubExternalWrenchesEstimationTest iDynTreeiCubExternalWrenchesEstimationTest.cpp)

#add_executable(iDynTreeCOM iDynTreeCOM.cpp)


target_link_libraries(iDynTreeJacobianTest idyntree-icub)
target_link_libraries(iDynTreeRelativeJacobianTest idyntree-icub)
target_link_libraries(iDynTreeWholeBodyFTsensorEstimateTest idyntree-icub)
target_link_libraries(iDynTreeExportFixedJointsFrames idyntree-icub)
target_link_libraries(iDynTreeExternalWrenchesEstimationTest idyntree-icub )
target_link_libraries(iDynTreeiCubExternalWrenchesEstimationTest idyntree-icub)

#target_link_libraries(iDynTreeCOM idyntree ${ICUB_LIBRARIES} ${kdl_codyco_LIBRARIES} )

add_test(NAME test_iDynTreeJacobianTest                COMMAND iDynTreeJacobianTest)
add_test(NAME test_iDynTreeRelativeJacobianTest        COMMAND iDynTreeRelativeJacobianTest)
add_test(NAME test_iDynTreeWholeBodyFTsensorEstimateTest COMMAND iDynTreeWholeBodyFTsensorEstimateTest)
add_test(NAME test_iDynTreeExportFixedJointsFrames     COMMAND iDynTreeExportFixedJointsFrames)
add_test(NAME test_iDynTreeExternalWrenchesEstimationTest COMMAND iDynTreeExternalWrenchesEstimationTest)
add_test(NAME test_iDynTreeiCubExternalWrenchesEstimationTest COMMAND iDynTreeiCubExternalWrenchesEstimationTest)


add_executable(iCubLeftArmRegressorTest iCubLeftArmRegressorTest.cpp)
target_link_libraries(iCubLeftArmRegressorTest idyntree-regressors idyntree-sensors idyntree-modelio-urdf ${urdfdom_LIBRARIES})
add_test(NAME test_iCubLeftArmRegressorTest COMMAND iCubLeftArmRegressorTest )