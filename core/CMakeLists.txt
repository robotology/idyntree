# Copyright: (C) 2015 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(iDynTree_Core CXX)

set( PROJ_SOURCE_DIR ${kdl_codyco_SOURCE_DIR} )
set( PROJ_BINARY_DIR ${kdl_codyco_BINARY_DIR} )

option(KDL_CODYCO_ENABLE_RPATH "Enable RPATH for the library" TRUE)
mark_as_advanced(KDL_CODYCO_ENABLE_RPATH)

if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
endif(MSVC)

include_directories(include ${orocos_kdl_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

include_directories(include
                    ./include/kdl_codyco/regressors ./include/kdl_codyco/regressors/dataset ${orocos_kdl_INCLUDE_DIRS})

file(GLOB  KDL_CODYCO_HPPS
     "include/kdl_codyco/*.hpp"
 )

set(IDYNTREE_CORE_HDRS include/kdl_codyco/treeserialization.hpp)

message("KDL_CODYCO_HPPS: ${KDL_CODYCO_HPPS}")

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

# share headers with all iDynTree targes
set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include")
get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)

include_directories(${IDYNTREE_TREE_INCLUDE_DIRS})

if(YCM_FOUND)
include(YCMDefaultDirs)

include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/bin"
                          LIB_DIRS "${CMAKE_INSTALL_PREFIX}/lib"
                          DEPENDS KDL_CODYCO_ENABLE_RPATH
                          USE_LINK_PATH)

endif()


add_library(idyntree-core ${KDL_CODYCO_SRC} ${DIRL_SRC} ${KDL_CODYCO_HPPS} ${KDL_CODYCO_PRIVATE_INCLUDES})

target_include_directories(idyntree-core PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
                                                "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

target_link_libraries(idyntree-core ${orocos_kdl_LIBRARIES})

set_target_properties(idyntree-core PROPERTIES
  SOVERSION "${KDL_CODYCO_VERSION_MAJOR}.${KDL_CODYCO_VERSION_MINOR}"
  VERSION "${KDL_CODYCO_VERSION}"
  COMPILE_FLAGS "${CMAKE_CXX_FLAGS_ADD}")

message(STATUS "papare  : " ${KDL_CODYCO_HPPS})
set_property(TARGET idyntree-core PROPERTY PUBLIC_HEADER ${IDYNTREE_CORE_HDRS})


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

install(TARGETS idyntree-core
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/kdl_codyco
        PRIVATE_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/kdl_codyco/impl)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS idyntree-core)


if(IDYNTREE_COMPILE_TESTS)
    add_subdirectory(tests)
endif(IDYNTREE_COMPILE_TESTS)

