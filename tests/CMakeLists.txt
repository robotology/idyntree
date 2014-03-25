include_directories(${EIGEN3_INCLUDE_DIR} ${Orocos-KDL_INCLUDE_DIRS} ${kdl_codyco_INCLUDE_DIRS} )

add_executable(inverse_inertia_test inverse_inertia.cpp)
add_executable(regressor_utils_test regressor_utils_test.cpp)
add_executable(subtree_regressor_test subtree_regressor_test.cpp testhumanoid.cpp)

target_link_libraries(inverse_inertia_test ${orocos_kdl_LIBRARIES} ${kdl_codyco_LIBRARIES})
target_link_libraries(regressor_utils_test ${orocos_kdl_LIBRARIES}  ${kdl_codyco_LIBRARIES})
target_link_libraries(subtree_regressor_test ${orocos_kdl_LIBRARIES}  ${kdl_codyco_LIBRARIES})

add_test(test_inverse_inertia inverse_inertia_test)
add_test(test_regressor_utils regressor_utils_test)
add_test(test_subtree_regressor subtree_regressor_test)
