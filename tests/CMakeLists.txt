
find_package(kdl_format_io)
include_directories(${CMAKE_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIR} ${orocos_kdl_INCLUDE_DIRS} ${iDynTree_INCLUDE_DIRS} ${kdl_codyco_INCLUDE_DIRS})

add_executable(iDynTreeJacobianTest iDynTreeJacobianTest.cpp)
if(IDYNTREE_BUILD_FT_TESTS)
add_executable(iDynTreeWholeBodyFTsensorEstimateTest iDynTreeWholeBodyFTsensorEstimateTest.cpp)
endif()
add_executable(iDynTreeExternalWrenchesEstimationTest iDynTreeExternalWrenchesEstimationTest.cpp)

target_link_libraries(iDynTreeJacobianTest idyntree ${ICUB_LIBRARIES} ${kdl_codyco_LIBRARIES} )
if(IDYNTREE_BUILD_FT_TESTS)
target_link_libraries(iDynTreeWholeBodyFTsensorEstimateTest idyntree ${ICUB_LIBRARIES} ${kdl_codyco_LIBRARIES})
endif()
target_link_libraries(iDynTreeExternalWrenchesEstimationTest idyntree ${ICUB_LIBRARIES} ${kdl_codyco_LIBRARIES}  )


add_test(NAME test_iDynTreeJacobianTest COMMAND iDynTreeJacobianTest)
if(IDYNTREE_BUILD_FT_TESTS)
add_test(NAME test_iDynTreeWholeBodyFTsensorEstimateTest COMMAND iDynTreeWholeBodyFTsensorEstimateTest)
endif()
add_test(NAME test_iDynTreeExternalWrenchesEstimationTest COMMAND iDynTreeExternalWrenchesEstimationTest)

