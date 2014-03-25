include_directories(${EIGEN3_INCLUDE_DIR} ${orocos_kdl_INCLUDE_DIRS} ${kdl_codyco_INCLUDE_DIRS} )

add_executable(inverse_inertia_test inverse_inertia.cpp)
add_executable(regressor_utils_test regressor_utils_test.cpp)
add_executable(subtree_regressor_test subtree_regressor_test.cpp testhumanoid.cpp)

target_link_libraries(inverse_inertia_test ${orocos_kdl_LIBRARIES} kdl-codyco)
target_link_libraries(regressor_utils_test ${orocos_kdl_LIBRARIES}  kdl-codyco)
target_link_libraries(subtree_regressor_test ${orocos_kdl_LIBRARIES}  kdl-codyco)

add_test(test_inverse_inertia inverse_inertia_test)
add_test(test_regressor_utils regressor_utils_test)
add_test(test_subtree_regressor subtree_regressor_test)
