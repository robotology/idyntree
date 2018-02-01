# Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

project(iDynTree_ModelIO_symoro CXX)

# add external parser
add_subdirectory(external/expression_parser)

include_directories(SYSTEM "${expression-parser_SOURCE_DIR}/include")


set(IDYNTREE_MODELIO_SYMORO_HEADERS include/iDynTree/ModelIO/symoro_par_import.hpp
                                    include/iDynTree/ModelIO/symoro_par_import_serialization.hpp
                                    include/iDynTree/ModelIO/symoro_par_model.hpp)

set(IDYNTREE_MODELIO_SYMORO_SOURCES src/symoro_par_import.cpp
                                    src/symoro_par_import_serialization.cpp)

source_group("Source Files" FILES ${IDYNTREE_MODELIO_SYMORO_SOURCES})
source_group("Header Files" FILES ${IDYNTREE_MODELIO_SYMORO_HEADERS})

set(libraryname idyntree-modelio-symoro)

add_library(${libraryname} ${IDYNTREE_MODELIO_SYMORO_SOURCES} ${IDYNTREE_MODELIO_SYMORO_HEADERS})

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>")

target_link_libraries(${libraryname} idyntree-kdl expression-parser ${orocos_kdl_LIBRARIES})

if(NOT CMAKE_VERSION VERSION_LESS 2.8.12)
    target_compile_options(${libraryname} PRIVATE ${IDYNTREE_WARNING_FLAGS})
endif()

include_directories(SYSTEM ${orocos_kdl_INCLUDE_DIRS})

set_property(TARGET ${libraryname} PROPERTY PUBLIC_HEADER ${IDYNTREE_MODELIO_SYMORO_HEADERS})

install(TARGETS ${libraryname}
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/ModelIO)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})
