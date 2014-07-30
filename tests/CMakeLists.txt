
find_package(kdl_format_io)
include_directories(${CMAKE_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIR} ${orocos_kdl_INCLUDE_DIRS} ${iDynTree_INCLUDE_DIRS} ${kdl_codyco_INCLUDE_DIRS})

add_executable(iDynTreeJacobianTest iDynTreeJacobianTest.cpp)
#add_executable(iDynTreeWholeBodyFTsensorEstimateTest iDynTreeWholeBodyFTsensorEstimateTest.cpp)
add_executable(iDynTreeExternalWrenchesEstimationTest iDynTreeExternalWrenchesEstimationTest.cpp)
#add_executable(iDynTreeCOM iDynTreeCOM.cpp)


target_link_libraries(iDynTreeJacobianTest idyntree ${ICUB_LIBRARIES} ${kdl_codyco_LIBRARIES} )
#target_link_libraries(iDynTreeWholeBodyFTsensorEstimateTest iDynTree ${ICUB_LIBRARIES} ${orocos_kdl_LIBRARIES} {kdl_codyco_LIBRARIES})
target_link_libraries(iDynTreeExternalWrenchesEstimationTest idyntree ${ICUB_LIBRARIES} ${orocos_kdl_LIBRARIES} ${kdl_codyco_LIBRARIES} )
#target_link_libraries(iDynTreeCOM idyntree ${ICUB_LIBRARIES} ${kdl_codyco_LIBRARIES} )

add_test(NAME test_iDynTreeJacobianTest COMMAND iDynTreeJacobianTest)
#add_test(NAME test_iDynTreeWholeBodyFTsensorEstimateTest COMMAND iDynTreeWholeBodyFTsensorEstimateTest)
add_test(NAME test_iDynTreeExternalWrenchesEstimationTest COMMAND iDynTreeExternalWrenchesEstimationTest)
