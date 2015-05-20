cmake_minimum_required( VERSION 2.8 FATAL_ERROR )
project (urdfdom_headers)

set (URDF_MAJOR_VERSION 0)
set (URDF_MINOR_VERSION 3)
set (URDF_PATCH_VERSION 0)

set (URDF_VERSION ${URDF_MAJOR_VERSION}.${URDF_MINOR_VERSION}.${URDF_PATCH_VERSION})

message (STATUS "${PROJECT_NAME} version ${URDF_VERSION}")

# This shouldn't be necessary, but there has been trouble
# with MSVC being set off, but MSVCXX ON.
if(MSVC OR MSVC90 OR MSVC10)
  set(MSVC ON)
endif (MSVC OR MSVC90 OR MSVC10)

add_subdirectory(urdf_sensor)
add_subdirectory(urdf_model)
add_subdirectory(urdf_model_state)
add_subdirectory(urdf_world)
add_subdirectory(urdf_exception)

set(PACKAGE_NAME ${PROJECT_NAME})
set(cmake_conf_file "cmake/${PROJECT_NAME}-config.cmake")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/${cmake_conf_file}.in" "${CMAKE_BINARY_DIR}/${cmake_conf_file}" @ONLY)
install(FILES "${CMAKE_BINARY_DIR}/${cmake_conf_file}" DESTINATION share/${PROJECT_NAME}/cmake/ COMPONENT cmake)

# Make the package config file
if (NOT MSVC)
  set(PACKAGE_DESC "Unified Robot Description Format")
  set(pkg_conf_file "cmake/pkgconfig/urdfdom_headers.pc")
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/${pkg_conf_file}.in" "${CMAKE_BINARY_DIR}/${pkg_conf_file}" @ONLY)
  install(FILES "${CMAKE_BINARY_DIR}/${pkg_conf_file}" DESTINATION lib/pkgconfig/ COMPONENT pkgconfig)
endif()

message(STATUS "Configuration successful. Type make install to install urdfdom_headers.")
