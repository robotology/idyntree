cmake_minimum_required( VERSION 2.8 FATAL_ERROR )
project (kdl_urdf CXX)

set (KDL_URDF_MAJOR_VERSION 0)
set (KDL_URDF_MINOR_VERSION 0)
set (KDL_URDF_PATCH_VERSION 1)

set (KDL_URDF_VERSION ${KDL_URDF_MAJOR_VERSION}.${KDL_URDF_MINOR_VERSION}.${KDL_URDF_PATCH_VERSION})

message (STATUS "${PROJECT_NAME} version ${KDL_URDF_VERSION}")

# set the default build type
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# This shouldn't be necessary, but there has been trouble
# with MSVC being set off, but MSVCXX ON.
if(MSVC OR MSVC90 OR MSVC10)
  set(MSVC ON)
endif (MSVC OR MSVC90 OR MSVC10)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(SearchForTinyXML)
include_directories(${tinyxml_include_dirs})
link_directories(${tinyxml_library_dirs})
add_definitions(${tinyxml_cflags})

find_package(urdfdom_headers REQUIRED)
include_directories(SYSTEM ${urdfdom_headers_INCLUDE_DIRS})

find_package(urdfdom REQUIRED)
include_directories(SYSTEM ${urdfdom_INCLUDE_DIRS})

find_package(console_bridge REQUIRED)
include_directories(SYSTEM ${console_bridge_INCLUDE_DIRS})
link_directories(${console_bridge_LIBRARY_DIRS})

find_package(Boost REQUIRED thread)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

add_subdirectory(kdl_import)
add_subdirectory(kdl_export)


set(PKG_NAME ${PROJECT_NAME})
set(PKG_LIBRARIES kdl_import kdl_export)
set(PKG_DEPENDS urdfdom urdfdom_headers console_bridge)
set(cmake_conf_file "${CMAKE_CURRENT_SOURCE_DIR}/cmake/kdl_urdf-config.cmake")
configure_file("${cmake_conf_file}.in" "${cmake_conf_file}" @ONLY)
install(FILES ${cmake_conf_file} DESTINATION share/${PROJECT_NAME}/cmake/ COMPONENT cmake)

# Make the package config file
if (NOT MSVC)
  set(PKG_DESC "Kinematics Dynamics Library support for Unified Robot Description Format")
  set(PKG_DEPENDS "urdfdom_headers console_bridge") # make the list separated by spaces instead of ;
  set(PKG_URDF_LIBS "-lkdl_import -lkdl_export")
  set(pkg_conf_file "${CMAKE_CURRENT_SOURCE_DIR}/cmake/pkgconfig/kdl_urdf.pc")
  configure_file("${pkg_conf_file}.in" "${pkg_conf_file}" @ONLY)
  install(FILES ${pkg_conf_file} DESTINATION lib/pkgconfig/ COMPONENT pkgconfig)
endif()


message(STATUS "Configuration successful. Type make to compile kdl_urdf")
