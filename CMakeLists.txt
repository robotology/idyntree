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

find_package(Orocos-KDL REQUIRED)
include_directories(SYSTEM  ${Orocos-KDL_INCLUDE_DIRS})
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

include_directories(include)

set(EXPR_PARSER_SRCS src/symoro_par/expression_parser/error.cpp
                     src/symoro_par/expression_parser/functions.cpp
                     src/symoro_par/expression_parser/parser.cpp
                     src/symoro_par/expression_parser/variablelist.cpp)

add_library(kdl_urdf SHARED src/urdf/kdl_export.cpp src/urdf/kdl_import.cpp 
                            src/symoro_par/symoro_par_import.cpp
                            ${EXPR_PARSER_SRCS})
                            
target_link_libraries(kdl_urdf  ${console_bridge_LIBRARIES} ${Boost_LIBRARIES} ${Orocos-KDL_LIBRARIES})

add_executable(check_kdl_import src/urdf/check_kdl_import.cpp src/urdf/kdl_import.cpp)
target_link_libraries(check_kdl_import ${urdfdom_headers_LIBRARIES}  ${urdfdom_LIBRARIES} ${console_bridge_LIBRARIES} ${Boost_LIBRARIES} ${Orocos-KDL_LIBRARIES})

add_executable(check_symoro_par_import src/symoro_par/check_symoro_par_import.cpp src/symoro_par/symoro_par_import.cpp ${EXPR_PARSER_SRCS})
target_link_libraries(check_symoro_par_import ${Orocos-KDL_LIBRARIES})

add_executable(par2urdf src/utils/par2urdf.cpp)
target_link_libraries(par2urdf kdl_urdf ${urdfdom_headers_LIBRARIES}  ${urdfdom_LIBRARIES} ${console_bridge_LIBRARIES} ${Boost_LIBRARIES} ${Orocos-KDL_LIBRARIES})

#add_executable(check_kdl_export src/check_kdl_export.cpp src/kdl_import.cpp src/kdl_export.cpp)
#target_link_libraries(check_kdl_export ${kdl_urdf_LIBRARIES} ${urdfdom_headers_LIBRARIES}  ${urdfdom_LIBRARIES} ${console_bridge_LIBRARIES} ${Boost_LIBRARIES} ${Orocos-KDL_LIBRARIES})


INSTALL(TARGETS kdl_urdf DESTINATION lib/)
INSTALL(DIRECTORY include/kdl_urdf DESTINATION include)



set(PKG_NAME ${PROJECT_NAME})
set(PKG_LIBRARIES kdl_urdf)
set(PKG_DEPENDS urdfdom urdfdom_headers console_bridge)
set(cmake_conf_file "${CMAKE_CURRENT_SOURCE_DIR}/cmake/kdl_urdf-config.cmake")
configure_file("${cmake_conf_file}.in" "${cmake_conf_file}" @ONLY)
install(FILES ${cmake_conf_file} DESTINATION share/${PROJECT_NAME}/cmake/ COMPONENT cmake)

# Make the package config file
if (NOT MSVC)
  set(PKG_DESC "Kinematics Dynamics Library support for Unified Robot Description Format")
  set(PKG_DEPENDS "Orocos-KDL urdfdom urdfdom_headers console_bridge") # make the list separated by spaces instead of ;
  set(PKG_URDF_LIBS "-lkdl_import -lkdl_export")
  set(pkg_conf_file "${CMAKE_CURRENT_SOURCE_DIR}/cmake/pkgconfig/kdl_urdf.pc")
  configure_file("${pkg_conf_file}.in" "${pkg_conf_file}" @ONLY)
  install(FILES ${pkg_conf_file} DESTINATION lib/pkgconfig/ COMPONENT pkgconfig)
endif()


message(STATUS "Configuration successful. Type make to compile kdl_urdf")
