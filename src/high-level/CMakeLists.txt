# Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

set(IDYNTREE_HIGH_LEVEL_HEADERS include/iDynTree/KinDynComputations.h)

set(IDYNTREE_HIGH_LEVEL_SOURCES src/KinDynComputations.cpp)

SOURCE_GROUP("Source Files" FILES ${IDYNTREE_HIGH_LEVEL_SOURCES})
SOURCE_GROUP("Header Files" FILES ${IDYNTREE_HIGH_LEVEL_HEADERS})

set(libraryname idyntree-high-level)

add_library(${libraryname} ${IDYNTREE_HIGH_LEVEL_SOURCES} ${IDYNTREE_HIGH_LEVEL_HEADERS} ${IDYNTREE_HIGH_LEVEL_PRIVATE_INCLUDES})
add_library(iDynTree::${libraryname} ALIAS ${libraryname})

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")

target_link_libraries(${libraryname} PUBLIC idyntree-core idyntree-model idyntree-sensors idyntree-modelio-urdf
                                     PRIVATE Eigen3::Eigen)

set_property(TARGET ${libraryname} PROPERTY PUBLIC_HEADER ${IDYNTREE_HIGH_LEVEL_HEADERS})

install(TARGETS ${libraryname}
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/
        PRIVATE_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})

if(IDYNTREE_COMPILE_TESTS)
    add_subdirectory(tests)
endif(IDYNTREE_COMPILE_TESTS)

