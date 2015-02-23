

find_package(TinyXML       REQUIRED)
include_directories(${CMAKE_SOURCE_DIR}/include
                          ${EIGEN3_INCLUDE_DIR}
                          ${orocos_kdl_INCLUDE_DIRS}
                          ${iDynTree_INCLUDE_DIRS}
                          ${YARP_INCLUDE_DIRS}
                          ${kdl_codyco_INCLUDE_DIRS}
                          ${ICUB_INCLUDE_DIRS})


configure_file(${CMAKE_CURRENT_SOURCE_DIR}/test_files/icub.urdf
               ${CMAKE_CURRENT_BINARY_DIR}/icub.urdf)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/test_files/bigman.urdf
               ${CMAKE_CURRENT_BINARY_DIR}/bigman.urdf)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/test_files/icub_model.urdf
               ${CMAKE_CURRENT_BINARY_DIR}/icub_model.urdf)
add_executable(iDynTreeJacobianTest         iDynTreeJacobianTest.cpp)
add_executable(iDynTreeRelativeJacobianTest iDynTreeRelativeJacobianTest.cpp)
add_executable(iDynTreeWholeBodyFTsensorEstimateTest iDynTreeWholeBodyFTsensorEstimateTest.cpp)
add_executable(iDynTreeExportFixedJointsFrames iDynTreeExportFixedJointsFrames.cpp)
add_executable(iDynTreeExternalWrenchesEstimationTest iDynTreeExternalWrenchesEstimationTest.cpp)

#add_executable(iDynTreeCOM iDynTreeCOM.cpp)


target_link_libraries(iDynTreeJacobianTest idyntree-yarp)
target_link_libraries(iDynTreeRelativeJacobianTest idyntree-yarp)
target_link_libraries(iDynTreeWholeBodyFTsensorEstimateTest ${orocos_kdl_LIBRARIES} idyntree-yarp)
target_link_libraries(iDynTreeExportFixedJointsFrames ${orocos_kdl_LIBRARIES} idyntree-yarp)
target_link_libraries(iDynTreeExternalWrenchesEstimationTest idyntree-yarp ${orocos_kdl_LIBRARIES} ${kdl_codyco_LIBRARIES} )
#target_link_libraries(iDynTreeCOM idyntree ${ICUB_LIBRARIES} ${kdl_codyco_LIBRARIES} )

add_test(NAME test_iDynTreeJacobianTest                COMMAND iDynTreeJacobianTest         icub.urdf)
add_test(NAME test_iDynTreeRelativeJacobianTest        COMMAND iDynTreeRelativeJacobianTest icub.urdf imu_frame)
add_test(NAME test_iDynTreeRelativeJacobianTest_bigman COMMAND iDynTreeRelativeJacobianTest bigman.urdf imu_link)
add_test(NAME test_iDynTreeWholeBodyFTsensorEstimateTest COMMAND iDynTreeWholeBodyFTsensorEstimateTest)
add_test(NAME test_iDynTreeExportFixedJointsFrames     COMMAND iDynTreeExportFixedJointsFrames)
add_test(NAME test_iDynTreeExternalWrenchesEstimationTest COMMAND iDynTreeExternalWrenchesEstimationTest)

add_executable(iCubLeftArmRegressorTest iCubLeftArmRegressorTest.cpp)
target_link_libraries(iCubLeftArmRegressorTest idyntree-regressors idyntree-model-io)
add_test(NAME test_iCubLeftArmRegressorTest COMMAND iCubLeftArmRegressorTest )