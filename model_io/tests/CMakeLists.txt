if(ENABLE_IKIN)
find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
endif()

include_directories(${Orocos-KDL_INCLUDE_DIRS} ${kdl_codyco_INCLUDE_DIRS} )

add_executable(check_urdf_import_export check_urdf_import_export.cpp)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/format_examples/urdf/black_icub.urdf ${CMAKE_CURRENT_BINARY_DIR}/black_icub.urdf)
target_link_libraries(check_urdf_import_export ${orocos_kdl_LIBRARIES} idyntree-core idyntree-model-io)
add_test(NAME test_urdf_import_export COMMAND check_urdf_import_export black_icub.urdf)

add_executable(check_symoro_par_import_fixed_chain_regressor check_symoro_par_import_fixed_chain_regressor.cpp)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/format_examples/symoro_par/fake_puma.par ${CMAKE_CURRENT_BINARY_DIR}/fake_puma.par)
target_link_libraries(check_symoro_par_import_fixed_chain_regressor  ${orocos_kdl_LIBRARIES} idyntree-core idyntree-model-io)
add_test(NAME test_par_import_fixed_chain_regressor COMMAND check_symoro_par_import_fixed_chain_regressor fake_puma.par)


#check iKin Denavit Hartenberg parameters export
#add_executable(check_iKin_export_random_chain check_iKin_export_random_chain.cpp)
#target_link_libraries(check_iKin_export_random_chain ${kdl_codyco_LIBRARIES} ${YARP_LIBRARIES} ${ICUB_LIBRARIES} kdl-format-io)


#In Release mode, don't compile the HRP2 Regressor model because it takes a lot of time
IF(CMAKE_BUILD_TYPE MATCHES Debug)
    add_executable(check_symoro_par_import_floating_tree_regressor check_symoro_par_import_floating_tree_base_regressor.cpp)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/format_examples/symoro_par/HRP2JRL_IMU.par ${CMAKE_CURRENT_BINARY_DIR}/HRP2JRL_IMU.par)
    target_link_libraries(check_symoro_par_import_floating_tree_regressor  ${orocos_kdl_LIBRARIES} idyntree-core idyntree-model-io)
    add_test(NAME test_par_import_floating_tree_regressor COMMAND check_symoro_par_import_floating_tree_regressor HRP2JRL_IMU.par)
ENDIF()

