# Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

project(iDynTree_Regressors CXX)

include_directories(include/kdl_codyco/regressors)
include_directories(include)

file(GLOB IDYNTREE_REGRESSORS_HEADERS
     "include/kdl_codyco/regressors/*.hpp"
 )

set(IDYNTREE_REGRESSOR_HEADERS_EXP include/iDynTree/Regressors/DynamicsRegressorParameters.h)

# TODO \todo : avoid globbing and explicitly state everithing
if (IDYNTREE_USES_KDL)
  file(GLOB IDYNTREE_REGRESSORS_SOURCES "src/*.cpp")
else()
  set(IDYNTREE_REGRESSORS_SOURCES src/DynamicsRegressorParameters.cpp src/dummy/DummyDynamicsRegressorGenerator.cpp)
endif()

set(IDYNTREE_REGRESSOR_SOURCES_EXP DynamicsRegressorParameters.cpp)

SOURCE_GROUP("Source Files" FILES ${IDYNTREE_REGRESSORS_SOURCES} ${IDYNTREE_REGRESSORS_SOURCES_EXP})
SOURCE_GROUP("Header Files" FILES ${IDYNTREE_REGRESSORS_HEADERS} ${IDYNTREE_REGRESSORS_HEADERS_EXP})

# Legacy library, remove deprecated warnings when compiling
idyntree_disable_deprecation_warnings()

set(libraryname idyntree-regressors)

add_library(${libraryname} ${IDYNTREE_REGRESSORS_SOURCES}     ${IDYNTREE_REGRESSORS_HEADERS}
                                ${IDYNTREE_REGRESSORS_SOURCES_EXP} ${IDYNTREE_REGRESSORS_HEADERS_EXP})

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>")
target_include_directories(${libraryname} PRIVATE SYSTEM ${orocos_kdl_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${TinyXML_INCLUDE_DIRS})

target_link_libraries(${libraryname} idyntree-core idyntree-sensors)
if (IDYNTREE_USES_KDL)
  target_link_libraries(${libraryname} idyntree-modelio-urdf-kdl ${orocos_kdl_LIBRARIES} ${TinyXML_LIBRARIES})
endif ()

# Ensure that build include directories are always included before system ones
get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)
target_include_directories(${libraryname} BEFORE PRIVATE ${IDYNTREE_TREE_INCLUDE_DIRS})

if((${CMAKE_CXX_COMPILER_ID} MATCHES "GNU") OR (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang"))
    target_compile_options(${libraryname} PRIVATE "-Wno-deprecated")
endif()

set_property(TARGET ${libraryname} PROPERTY PUBLIC_HEADER ${IDYNTREE_REGRESSORS_HEADERS})

install(TARGETS ${libraryname}
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/kdl_codyco/regressors)

# Install headers in iDynTree/Regressors directory
install(DIRECTORY include/iDynTree/Regressors DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})

get_property(IDYNTREE_COMP_INCLUDE_DIRS TARGET ${libraryname} PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS ${IDYNTREE_COMP_INCLUDE_DIRS})
