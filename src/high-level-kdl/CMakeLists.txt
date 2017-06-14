# Copyright: (C) 2015 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(iDynTree_HighLevel_KDL CXX)

set(IDYNTREE_HIGH_LEVEL_KDL_HEADERS include/iDynTree/HighLevel/DynamicsComputations.h)

set(IDYNTREE_HIGH_LEVEL_KDL_PRIVATE_INCLUDES )

if (IDYNTREE_USES_KDL)
    set(IDYNTREE_HIGH_LEVEL_KDL_SOURCES src/DynamicsComputations.cpp)
else ()
    set(IDYNTREE_HIGH_LEVEL_KDL_SOURCES src/DummyDynamicsComputations.cpp)
endif()

SOURCE_GROUP("Source Files" FILES ${IDYNTREE_HIGH_LEVEL_KDL_SOURCES})
SOURCE_GROUP("Header Files" FILES ${IDYNTREE_HIGH_LEVEL_KDL_HEADERS})

set(libraryname idyntree-high-level-kdl)

add_library(${libraryname} ${IDYNTREE_HIGH_LEVEL_KDL_SOURCES} ${IDYNTREE_HIGH_LEVEL_KDL_HEADERS} ${IDYNTREE_HIGH_LEVEL_KDL_PRIVATE_INCLUDES})

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>")

if((${CMAKE_CXX_COMPILER_ID} MATCHES "GNU") OR (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang"))
    target_compile_options(${libraryname} PRIVATE "-Wno-deprecated")
endif()

include_directories(${libraryname} SYSTEM ${EIGEN3_INCLUDE_DIR} ${orocos_kdl_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS})

target_link_libraries(${libraryname} idyntree-core)
if (IDYNTREE_USES_KDL)
    target_link_libraries(${libraryname} idyntree-kdl idyntree-modelio-urdf-kdl idyntree-sensors idyntree-modelio-urdf-kdl ${TinyXML_LIBRARIES})
endif()


# Ensure that build include directories are always included before system ones
get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)
target_include_directories(${libraryname} BEFORE PRIVATE ${IDYNTREE_TREE_INCLUDE_DIRS})

set_property(TARGET ${libraryname} PROPERTY PUBLIC_HEADER ${IDYNTREE_HIGH_LEVEL_KDL_HEADERS})

install(TARGETS ${libraryname}
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/HighLevel
        PRIVATE_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/HighLevel/impl)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})

get_property(IDYNTREE_COMP_INCLUDE_DIRS TARGET ${libraryname} PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS ${IDYNTREE_COMP_INCLUDE_DIRS})

if(IDYNTREE_COMPILE_TESTS AND IDYNTREE_USES_KDL)
    add_subdirectory(tests)
endif()
