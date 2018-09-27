# Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

project(iDynTree_ModelIO_XML CXX)

find_package(LibXml2 REQUIRED)

set(IDYNTREE_MODELIO_XML_HEADERS include/iDynTree/XMLParser.h
                                 include/iDynTree/XMLElement.h
                                 include/iDynTree/XMLAttribute.h
                                 include/iDynTree/XMLDocument.h)

set(IDYNTREE_MODELIO_XML_PRIVATE_HEADERS )

set(IDYNTREE_MODELIO_XML_SOURCES src/XMLParser.cpp
                                 src/XMLElement.cpp
                                 src/XMLAttribute.cpp
                                 src/XMLDocument.cpp)


SOURCE_GROUP("Source Files" FILES ${IDYNTREE_MODELIO_XML_SOURCES})
SOURCE_GROUP("Header Files" FILES ${IDYNTREE_MODELIO_XML_HEADERS})
SOURCE_GROUP("Private Header Files" FILES ${IDYNTREE_MODELIO_XML_PRIVATE_HEADERS})

# share headers with all iDynTree targets
set(libraryname idyntree-modelio-xml)

add_library(${libraryname} ${IDYNTREE_MODELIO_XML_SOURCES} ${IDYNTREE_MODELIO_XML_HEADERS} ${IDYNTREE_MODELIO_XML_PRIVATE_HEADERS})

# Test if this works:
# We want to include the same-library header files directly (in the implementation), but with the full prefix in the .h (as this will be public)
target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/iDynTree>"
												 "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/private>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>"
                                                  ${IDYNTREE_TREE_INCLUDE_DIRS}
                                          PRIVATE ${EIGEN3_INCLUDE_DIR}
                                                  ${LIBXML2_INCLUDE_DIR})

target_link_libraries(${libraryname} LINK_PUBLIC 
                                     LINK_PRIVATE ${LIBXML2_LIBRARIES} idyntree-core)
                                     
if(NOT CMAKE_VERSION VERSION_LESS 2.8.12)
    target_compile_options(${libraryname} PRIVATE ${IDYNTREE_WARNING_FLAGS} ${LIBXML2_DEFINITIONS})
endif()

# Ensure that build include directories are always included before system ones
get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)
target_include_directories(${libraryname} BEFORE PRIVATE ${IDYNTREE_TREE_INCLUDE_DIRS})

set_property(TARGET ${libraryname} PROPERTY PUBLIC_HEADER ${IDYNTREE_MODELIO_XML_HEADERS})

install(TARGETS ${libraryname}
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})

get_property(IDYNTREE_COMP_INCLUDE_DIRS TARGET ${libraryname} PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS ${IDYNTREE_COMP_INCLUDE_DIRS})

if(IDYNTREE_COMPILE_TESTS)
    add_subdirectory(tests)
endif(IDYNTREE_COMPILE_TESTS)
