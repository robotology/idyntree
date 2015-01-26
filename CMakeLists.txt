#
# Test CMake version
#
cmake_minimum_required( VERSION 2.8 FATAL_ERROR )
set(PROJECT_NAME kdl_codyco)
project (${PROJECT_NAME} CXX)

set (KDL_CODYCO_VERSION_MAJOR 0)
set (KDL_CODYCO_VERSION_MINOR 2)
set (KDL_CODYCO_VERSION_PATCH 5)

set (KDL_CODYCO_VERSION ${KDL_CODYCO_VERSION_MAJOR}.${KDL_CODYCO_VERSION_MINOR}.${KDL_CODYCO_VERSION_PATCH})

message (STATUS "${PROJECT_NAME} version ${KDL_CODYCO_VERSION}")

set( PROJ_SOURCE_DIR ${kdl_codyco_SOURCE_DIR} )
set( PROJ_BINARY_DIR ${kdl_codyco_BINARY_DIR} )

option(KDL_CODYCO_ENABLE_RPATH "Enable RPATH for the library" TRUE)
mark_as_advanced(KDL_CODYCO_ENABLE_RPATH)

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

find_package(Eigen3 REQUIRED)

include_directories(include ${orocos_kdl_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

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

find_package(YCM)
if(YCM_FOUND)
include(YCMDefaultDirs)

include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/bin"
                          LIB_DIRS "${CMAKE_INSTALL_PREFIX}/lib"
                          DEPENDS KDL_CODYCO_ENABLE_RPATH
                          USE_LINK_PATH)

endif()

add_library(kdl-codyco ${LIB_TYPE} ${KDL_CODYCO_SRC} ${DIRL_SRC} ${KDL_CODYCO_HPPS} ${KDL_CODYCO_PRIVATE_INCLUDES})

target_link_libraries(kdl-codyco ${orocos_kdl_LIBRARIES})

set_target_properties(kdl-codyco PROPERTIES
  SOVERSION "${KDL_CODYCO_VERSION_MAJOR}.${KDL_CODYCO_VERSION_MINOR}"
  VERSION "${KDL_CODYCO_VERSION}"
  COMPILE_FLAGS "${CMAKE_CXX_FLAGS_ADD}"
  PUBLIC_HEADER "${KDL_CODYCO_HPPS}"
  )

if(${CMAKE_MINIMUM_REQUIRED_VERSION} VERSION_GREATER "2.8.12")
    message(AUTHOR_WARNING "CMAKE_MINIMUM_REQUIRED_VERSION is now ${CMAKE_MINIMUM_REQUIRED_VERSION}. This check can be removed.")
endif()
if(CMAKE_VERSION VERSION_LESS 2.8.12)
    if(NOT KDL_CODYCO_DISABLE_RPATH)
        set_target_properties(kdl-codyco PROPERTIES
            INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
        )
    endif(NOT KDL_CODYCO_DISABLE_RPATH)
endif(CMAKE_VERSION VERSION_LESS 2.8.12)

install(TARGETS kdl-codyco
  EXPORT kdl_codyco-target
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  PUBLIC_HEADER DESTINATION include/kdl_codyco
)

install(DIRECTORY include/kdl_codyco/regressors DESTINATION include/kdl_codyco)
install(DIRECTORY include/kdl_codyco/regressors/dataset DESTINATION include/kdl_codyco)


option(CREATE_TESTS "Create regression tests using CMake" FALSE)

if(CREATE_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif(CREATE_TESTS)

export(TARGETS kdl-codyco FILE "${PROJECT_BINARY_DIR}/kdl_codyco-target.cmake")

export(PACKAGE kdl_codyco)

configure_file(cmake/kdl_codyco-config.cmake.in ${PROJECT_BINARY_DIR}/kdl_codyco-config.cmake @ONLY)
configure_file(cmake/kdl_codyco-config-version.cmake.in ${PROJECT_BINARY_DIR}/kdl_codyco-config-version.cmake @ONLY)

install(FILES ${PROJECT_BINARY_DIR}/kdl_codyco-config.cmake DESTINATION share/kdl_codyco)
install(FILES ${PROJECT_BINARY_DIR}/kdl_codyco-config-version.cmake DESTINATION share/kdl_codyco)
install(EXPORT kdl_codyco-target  DESTINATION share/kdl_codyco)

message(STATUS "Configuration successful. Type make to compile kdl_codyco")

add_subdirectory(doc)

include(AddUninstallTarget)
