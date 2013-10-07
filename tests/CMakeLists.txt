find_package(kdl_codyco REQUIRED)



include_directories(${Orocos-KDL_INCLUDE_DIRS} ${kdl_codyco_INCLUDE_DIRS} )

add_executable(check_urdf_import_export check_urdf_import_export.cpp)
message("installing urdf from " ${CMAKE_CURRENT_SOURCE_DIR}/format_examples/urdf/black_icub.urdf to ${CMAKE_CURRENT_BINARY_DIR}/black_icub.urdf)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/format_examples/urdf/black_icub.urdf ${CMAKE_CURRENT_BINARY_DIR}/black_icub.urdf)

target_link_libraries(check_urdf_import_export ${Orocos-KDL_LIBRARIES} kdl_codyco kdl_format_io)

add_test(test_urdf_import_export check_urdf_import_export black_icub.urdf)