cmake_minimum_required( VERSION 2.8 FATAL_ERROR )
project (kdl_codyco CXX)

set (KDL_CODYCO_MAJOR_VERSION 0)
set (KDL_CODYCO_MINOR_VERSION 0)
set (KDL_CODYCO_PATCH_VERSION 1)

set (KDL_CODYCO_VERSION ${KDL_CODYCO_MAJOR_VERSION}.${KDL_CODYCO_MINOR_VERSION}.${KDL_CODYCO_PATCH_VERSION})

message (STATUS "${PROJECT_NAME} version ${KDL_CODYCO_VERSION}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

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

find_package(Eigen3 REQUIRED)
find_package(Orocos-KDL REQUIRED)

set(PKG_NAME ${PROJECT_NAME})
set(PKG_LIBRARIES kdl_codyco)
set(PKG_DEPENDS )
set(cmake_conf_file "${CMAKE_CURRENT_SOURCE_DIR}/cmake/kdl_codyco-config.cmake")
configure_file("${cmake_conf_file}.in" "${cmake_conf_file}" @ONLY)
install(FILES ${cmake_conf_file} DESTINATION share/${PROJECT_NAME}/cmake/ COMPONENT cmake)

# Make the package config file
if (NOT MSVC)
  set(PKG_DESC "Kinematics Dynamics Library additional solvers by CoDyCo project")
  set(PKG_DEPENDS "Orocos-KDL eigen3") # make the list separated by spaces instead of ;
  set(PKG_LIBS "")
  set(pkg_conf_file "${CMAKE_CURRENT_SOURCE_DIR}/cmake/pkgconfig/kdl_codyco.pc")
  configure_file("${pkg_conf_file}.in" "${pkg_conf_file}" @ONLY)
  install(FILES ${pkg_conf_file} DESTINATION lib/pkgconfig/ COMPONENT pkgconfig)
endif()

include_directories(include ${EIGEN3_INCLUDE_DIR} ${Orocos-KDL_INCLUDE_DIRS})

file(GLOB KDL_CODYCO_SRC
    "src/*.cpp"
)

add_library(kdl_codyco SHARED ${KDL_CODYCO_SRC})
target_link_libraries(kdl_codyco ${Orocos-KDL_LIBRARIES})

# --------------------------------

INSTALL(TARGETS kdl_codyco DESTINATION lib/)
INSTALL(DIRECTORY include/kdl_codyco DESTINATION include)

message(STATUS "Configuration successful. Type make to compile kdl_codyco")
