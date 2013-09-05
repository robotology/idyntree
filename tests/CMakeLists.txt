set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(Eigen3 REQUIRED)
find_package(Orocos-KDL REQUIRED)
find_package(kdl_codyco REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR} ${Orocos-KDL_INCLUDE_DIRS} ${kdl_codyco_INCLUDE_DIRS} )

add_executable(inverse_inertia_test inverse_inertia.cpp)

target_link_libraries(inverse_inertia_test ${Orocos-KDL_LIBRARIES} ${kdl_codyco_LIBRARIES} )

add_test(test_inverse_inertia inverse_inertia_test)