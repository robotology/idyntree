find_package(kdl_codyco REQUIRED)



include_directories(${Orocos-KDL_INCLUDE_DIRS} ${kdl_codyco_INCLUDE_DIRS} )

add_executable(check_urdf_import_export check_urdf_import_export.cpp)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/format_examples/urdf/black_icub.urdf ${CMAKE_CURRENT_BINARY_DIR}/black_icub.urdf)
target_link_libraries(check_urdf_import_export ${Orocos-KDL_LIBRARIES} kdl_codyco kdl_format_io)
add_test(test_urdf_import_export check_urdf_import_export black_icub.urdf)

add_executable(check_symoro_par_import_fixed_chain_regressor check_symoro_par_import_fixed_chain_regressor.cpp)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/format_examples/symoro_par/fake_puma.par ${CMAKE_CURRENT_BINARY_DIR}/fake_puma.par)
target_link_libraries(check_symoro_par_import_fixed_chain_regressor ${Orocos-KDL_LIBRARIES} kdl_codyco kdl_format_io)
add_test(test_par_import_fixed_chain_regressor check_symoro_par_import_fixed_chain_regressor fake_puma.par)

#In Release mode, don't compile the HRP2 Regressor model because it takes a lot of time
IF(CMAKE_BUILD_TYPE MATCHES Debug)
    add_executable(check_symoro_par_import_floating_tree_regressor check_symoro_par_import_floating_tree_base_regressor.cpp)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/format_examples/symoro_par/HRP2JRL_IMU.par ${CMAKE_CURRENT_BINARY_DIR}/HRP2JRL_IMU.par)
    target_link_libraries(check_symoro_par_import_floating_tree_regressor ${Orocos-KDL_LIBRARIES} kdl_codyco kdl_format_io)
    add_test(test_par_import_floating_tree_regressor check_symoro_par_import_floating_tree_regressor HRP2JRL_IMU.par)
ENDIF()

