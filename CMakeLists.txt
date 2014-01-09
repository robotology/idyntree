cmake_minimum_required( VERSION 2.8 FATAL_ERROR )
set(PROJECT_NAME kdl_codyco)
project (${PROJECT_NAME} CXX)

set (KDL_CODYCO_VERSION_MAJOR 0)
set (KDL_CODYCO_VERSION_MINOR 0)
set (KDL_CODYCO_VERSION_PATCH 2)

set (KDL_CODYCO_VERSION ${KDL_CODYCO_VERSION_MAJOR}.${KDL_CODYCO_VERSION_MINOR}.${KDL_CODYCO_VERSION_PATCH})

message (STATUS "${PROJECT_NAME} version ${KDL_CODYCO_VERSION}")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
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


include_directories(include ${orocos_kdl_INCLUDE_DIRS})

include_directories(include ./include/dirl ./include/dirl/dataset ${orocos_kdl_INCLUDE_DIRS})

file(GLOB KDL_CODYCO_HPPS
     "include/kdl_codyco/*.hpp"
)

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

SET_TARGET_PROPERTIES( kdl_codyco PROPERTIES
  SOVERSION "${KDL_CODYCO_VERSION_MAJOR}.${KDL_CODYCO_VERSION_MINOR}"
  VERSION "${KDL_CODYCO_VERSION}"
  COMPILE_FLAGS "${CMAKE_CXX_FLAGS_ADD}"
  INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
  PUBLIC_HEADER "${KDL_CODYCO_HPPS}"
  )


INSTALL(TARGETS kdl_codyco
  EXPORT kdl_codyco-target
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  PUBLIC_HEADER DESTINATION include/kdl_codyco
)

# --------------------------------
INSTALL(DIRECTORY include/kdl_codyco DESTINATION include) 
INSTALL(DIRECTORY include/dirl       DESTINATION include)
INSTALL(DIRECTORY include/dirl/dataset       DESTINATION include/dirl)

option(CREATE_TESTS "Create regression tests using CMake" FALSE)

IF(CREATE_TESTS)
    enable_testing()
    ADD_SUBDIRECTORY(tests)
ENDIF(CREATE_TESTS)

export(TARGETS kdl_codyco FILE "${PROJECT_BINARY_DIR}/kdl_codyco-target.cmake")

EXPORT(PACKAGE kdl_codyco)

CONFIGURE_FILE(cmake/kdl_codyco-config.cmake.in
  ${PROJECT_BINARY_DIR}/kdl_codyco-config.cmake @ONLY)

set(cmake_conf_file "${PROJECT_BINARY_DIR}/kdl_codyco-config.cmake")
INSTALL(FILES ${cmake_conf_file} DESTINATION share/kdl_codyco)
INSTALL(EXPORT kdl_codyco-target  DESTINATION share/kdl_codyco)

message(STATUS "Configuration successful. Type make to compile kdl_codyco")

add_subdirectory(doc)
