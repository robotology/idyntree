
find_package(kdl_format_io)
include_directories(${CMAKE_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIR} ${orocos_kdl_INCLUDE_DIRS} ${iDynTree_INCLUDE_DIRS} ${kdl_codyco_INCLUDE_DIRS})


configure_file(${CMAKE_CURRENT_SOURCE_DIR}/test_files/icub.urdf
               ${CMAKE_CURRENT_BINARY_DIR}/icub.urdf)
add_executable(iDynTreeJacobianTest         iDynTreeJacobianTest.cpp)
add_executable(iDynTreeRelativeJacobianTest iDynTreeRelativeJacobianTest.cpp)
#add_executable(iDynTreeWholeBodyFTsensorEstimateTest iDynTreeWholeBodyFTsensorEstimateTest.cpp)
#add_executable(iDynTreeExternalWrenchesEstimationTest iDynTreeExternalWrenchesEstimationTest.cpp)
#add_executable(iDynTreeCOM iDynTreeCOM.cpp)


target_link_libraries(iDynTreeJacobianTest idyntree ${ICUB_LIBRARIES} ${kdl_codyco_LIBRARIES} ${kdl_format_io_LIBRARIES})
target_link_libraries(iDynTreeRelativeJacobianTest idyntree ${ICUB_LIBRARIES} ${kdl_codyco_LIBRARIES} ${kdl_format_io_LIBRARIES})

#target_link_libraries(iDynTreeWholeBodyFTsensorEstimateTest iDynTree ${ICUB_LIBRARIES} ${orocos_kdl_LIBRARIES} {kdl_codyco_LIBRARIES})
#target_link_libraries(iDynTreeExternalWrenchesEstimationTest idyntree ${ICUB_LIBRARIES} ${orocos_kdl_LIBRARIES} ${kdl_codyco_LIBRARIES} )
#target_link_libraries(iDynTreeCOM idyntree ${ICUB_LIBRARIES} ${kdl_codyco_LIBRARIES} )

add_test(NAME test_iDynTreeJacobianTest COMMAND iDynTreeJacobianTest icub.urdf)
add_test(NAME test_iDynTreeRelativeJacobianTest COMMAND iDynTreeRelativeJacobianTest icub.urdf)

#add_test(NAME test_iDynTreeWholeBodyFTsensorEstimateTest COMMAND iDynTreeWholeBodyFTsensorEstimateTest)
#add_test(NAME test_iDynTreeExternalWrenchesEstimationTest COMMAND iDynTreeExternalWrenchesEstimationTest)
