#
# Test CMake version
#
cmake_minimum_required( VERSION 2.8 FATAL_ERROR )
set(PROJECT_NAME kdl_codyco)
project (${PROJECT_NAME} CXX)

set (KDL_CODYCO_VERSION_MAJOR 0)
set (KDL_CODYCO_VERSION_MINOR 1)
set (KDL_CODYCO_VERSION_PATCH 2)

set (KDL_CODYCO_VERSION ${KDL_CODYCO_VERSION_MAJOR}.${KDL_CODYCO_VERSION_MINOR}.${KDL_CODYCO_VERSION_PATCH})

message (STATUS "${PROJECT_NAME} version ${KDL_CODYCO_VERSION}")

SET( PROJ_SOURCE_DIR ${kdl_codyco_SOURCE_DIR} )
SET( PROJ_BINARY_DIR ${kdl_codyco_BINARY_DIR} )

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

find_package(orocos_kdl 1.2.3 QUIET)
if (NOT orocos_kdl_FOUND)
    SET(OROCOS_KDL_OLDVERSION On)
    find_package(orocos_kdl)
endif(NOT orocos_kdl_FOUND)

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
      set(orocos_kdl_VERSION ${Orocos-KDL_VERSION})
      set(orocos_kdl_VERSION_MAJOR ${Orocos-KDL_VERSION_MAJOR})
      set(orocos_kdl_VERSION_MINOR ${Orocos-KDL_VERSION_MINOR})
      set(orocos_kdl_VERSION_PATCH ${Orocos-KDL_VERSION_PATCH})
   endif(NOT Orocos-KDL_FOUND)
endif(NOT orocos_kdl_FOUND)

add_definitions(-DOROCOS_KDL_VERSION_MAJOR=${orocos_kdl_VERSION_MAJOR})
add_definitions(-DOROCOS_KDL_VERSION_MINOR=${orocos_kdl_VERSION_MINOR})
add_definitions(-DOROCOS_KDL_VERSION_PATCH=${orocos_kdl_VERSION_PATCH})


# if (OROCOS_KDL_OLDVERSION)
#     configure_file(include/kdl_codyco/config.h.in ${CMAKE_SOURCE_DIR}/include/kdl_codyco/config.h)
#     message(STATUS "Old orocos kdl version. Adding include file to define compatibility macros.")
# else(OROCOS_KDL_OLDVERSION)
#     file(WRITE ./include/kdl_codyco/config.h "/**\n * Copyright  (C) 2013  CoDyCo Project\n * Author: Silvio Traversaro\n * website: http://www.codyco.eu\n */ ")
# endif (OROCOS_KDL_OLDVERSION)

include_directories(include ${orocos_kdl_INCLUDE_DIRS})

include_directories(include ./include/kdl_codyco/regressors ./include/kdl_codyco/regressors/dataset ${orocos_kdl_INCLUDE_DIRS})

file(GLOB KDL_CODYCO_HPPS
     "include/kdl_codyco/*.hpp"
 )

file (GLOB KDL_CODYCO_PRIVATE_INCLUDES
    "include/kdl_codyco/config.h"
)

file(GLOB KDL_CODYCO_SRC
    "src/*.cpp"
)

file(GLOB DIRL_SRC
    "src/regressors/*.cpp" "src/regressors/dataset/*.cpp"
)

SOURCE_GROUP("Source Files" FILES ${KDL_CODYCO_SRC} ${DIRL_SRC})
SOURCE_GROUP("Header Files" FILES ${KDL_CODYCO_HPPS} ${KDL_CODYCO_PRIVATE_INCLUDES})


if(MSVC)
    set(LIB_TYPE STATIC)
else()
    set(LIB_TYPE SHARED)
endif()

add_library(kdl-codyco ${LIB_TYPE} ${KDL_CODYCO_SRC} ${DIRL_SRC} ${KDL_CODYCO_HPPS} ${KDL_CODYCO_PRIVATE_INCLUDES})

target_link_libraries(kdl-codyco ${orocos_kdl_LIBRARIES})

SET_TARGET_PROPERTIES(kdl-codyco PROPERTIES
  SOVERSION "${KDL_CODYCO_VERSION_MAJOR}.${KDL_CODYCO_VERSION_MINOR}"
  VERSION "${KDL_CODYCO_VERSION}"
  COMPILE_FLAGS "${CMAKE_CXX_FLAGS_ADD}"
  INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
  PUBLIC_HEADER "${KDL_CODYCO_HPPS}"
  )


INSTALL(TARGETS kdl-codyco
  EXPORT kdl_codyco-target
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  PUBLIC_HEADER DESTINATION include/kdl_codyco
)

INSTALL(DIRECTORY include/kdl_codyco/regressors DESTINATION include/kdl_codyco)
INSTALL(DIRECTORY include/kdl_codyco/regressors/dataset DESTINATION include/kdl_codyco)


OPTION(CREATE_TESTS "Create regression tests using CMake" FALSE)

IF(CREATE_TESTS)
    enable_testing()
    ADD_SUBDIRECTORY(tests)
ENDIF(CREATE_TESTS)

EXPORT(TARGETS kdl-codyco FILE "${PROJECT_BINARY_DIR}/kdl_codyco-target.cmake")

EXPORT(PACKAGE kdl_codyco)

CONFIGURE_FILE(cmake/kdl_codyco-config.cmake.in ${PROJECT_BINARY_DIR}/kdl_codyco-config.cmake @ONLY)
CONFIGURE_FILE(cmake/kdl_codyco-config-version.cmake.in ${PROJECT_BINARY_DIR}/kdl_codyco-config-version.cmake @ONLY)

INSTALL(FILES ${PROJECT_BINARY_DIR}/kdl_codyco-config.cmake DESTINATION share/kdl_codyco)
INSTALL(FILES ${PROJECT_BINARY_DIR}/kdl_codyco-config-version.cmake DESTINATION share/kdl_codyco)
INSTALL(EXPORT kdl_codyco-target  DESTINATION share/kdl_codyco)

message(STATUS "Configuration successful. Type make to compile kdl_codyco")

add_subdirectory(doc)

include(AddUninstallTarget)
