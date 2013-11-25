cmake_minimum_required( VERSION 2.8 FATAL_ERROR )
set(PROJECT_NAME kdl_codyco)
project (${PROJECT_NAME} CXX)

set (KDL_CODYCO_VERSION_MAJOR 0)
set (KDL_CODYCO_VERSION_MINOR 0)
set (KDL_CODYCO_VERSION_PATCH 2)

set (KDL_CODYCO_VERSION ${KDL_CODYCO_VERSION_MAJOR}.${KDL_CODYCO_VERSION_MINOR}.${KDL_CODYCO_VERSION_PATCH})

message (STATUS "${PROJECT_NAME} version ${KDL_CODYCO_VERSION}")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

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

if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
endif(MSVC) 

find_package(orocos_kdl)

#support also for the old version of kdl cmake package
if(NOT orocos_kdl_FOUND)
   find_package(Orocos-KDL)
   if(NOT Orocos-KDL_FOUND) 
      message(WARNING "KDL not found: neither orocos_kdl or Orocos-KDL cmake packages are available")
   else(NOT Orocos-KDL_FOUND)
      set(orocos_kdl_INCLUDE_DIRS ${Orocos-KDL_INCLUDE_DIRS})
      set(orocos_kdl_LIBRARY_DIRS ${Orocos-KDL_LIBRARY_DIRS})
      set(orocos_kdl_LIBRARIES ${Orocos-KDL_LIBRARIES})
      set(orocos_kdl_FOUND true)
   endif(NOT Orocos-KDL_FOUND)
endif(NOT orocos_kdl_FOUND)

set(PKG_NAME ${PROJECT_NAME})
set(PKG_LIBRARIES kdl_codyco)
set(PKG_DEPENDS )

INSTALL(EXPORT kdl_codyco-target DESTINATION "share/${PROJECT_NAME}") 

set(cmake_conf_file "${CMAKE_CURRENT_SOURCE_DIR}/cmake/kdl_codyco-config.cmake")
configure_file("${cmake_conf_file}.in" "${cmake_conf_file}" @ONLY)
install(FILES ${cmake_conf_file} DESTINATION share/${PROJECT_NAME}/cmake/ COMPONENT cmake)
INSTALL(FILES ${cmake_conf_file} DESTINATION share/${PROJECT_NAME})
INSTALL(FILES ${cmake_conf_file} DESTINATION ${CMAKE_INSTALL_PREFIX})


# Make the package config file
if (NOT MSVC)
  set(PKG_DESC "Kinematics Dynamics Library additional solvers by CoDyCo project")
  set(PKG_DEPENDS "orocos-kdl eigen3") # make the list separated by spaces instead of ;
  set(PKG_LIBS "")
  set(pkg_conf_file "${CMAKE_CURRENT_SOURCE_DIR}/cmake/pkgconfig/kdl_codyco.pc")
  configure_file("${pkg_conf_file}.in" "${pkg_conf_file}" @ONLY)
  install(FILES ${pkg_conf_file} DESTINATION lib/pkgconfig/ COMPONENT pkgconfig)
endif()

include_directories(include ./include/dirl ./include/dirl/dataset ${orocos_kdl_INCLUDE_DIRS})

file(GLOB KDL_CODYCO_SRC
    "src/*.cpp"
)

file(GLOB DIRL_SRC 
    "src/dirl/*.cpp" "src/dirl/dataset/*.cpp"
)

if(MSVC)
    set(LIB_TYPE STATIC)
else()
    set(LIB_TYPE SHARED)
endif()

add_library(kdl_codyco ${LIB_TYPE} ${KDL_CODYCO_SRC} ${DIRL_SRC})

target_link_libraries(kdl_codyco ${orocos_kdl_LIBRARIES})

# --------------------------------
INSTALL(TARGETS kdl_codyco EXPORT kdl_codyco-target DESTINATION lib)
INSTALL(DIRECTORY include/kdl_codyco DESTINATION include) 
INSTALL(DIRECTORY include/dirl       DESTINATION include)
INSTALL(DIRECTORY include/dirl/dataset       DESTINATION include/dirl)



option(CREATE_TESTS "Create regression tests using CMake" FALSE)

IF(CREATE_TESTS)
    enable_testing()
    ADD_SUBDIRECTORY(tests)
ENDIF(CREATE_TESTS)

message(STATUS "Configuration successful. Type make to compile kdl_codyco")

add_subdirectory(doc)
