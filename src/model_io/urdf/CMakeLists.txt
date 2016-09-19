# Copyright: (C) 2015 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(iDynTree_ModelIO_URDF CXX)

set(IDYNTREE_MODELIO_URDF_HEADERS include/iDynTree/ModelIO/URDFModelImport.h
                                  include/iDynTree/ModelIO/URDFDofsImport.h
                                  include/iDynTree/ModelIO/URDFGenericSensorsImport.h
                                  include/iDynTree/ModelIO/ModelLoader.h)

set(IDYNTREE_MODELIO_URDF_SOURCES src/URDFParsingUtils.h
                                  src/URDFModelImport.cpp
                                  src/URDFDofsImport.cpp
                                  src/URDFGenericSensorsImport.cpp
                                  src/URDFSolidShapesImport.cpp
                                  src/ModelLoader.cpp)

SOURCE_GROUP("Source Files" FILES ${IDYNTREE_MODELIO_URDF_SOURCES})
SOURCE_GROUP("Header Files" FILES ${IDYNTREE_MODELIO_URDF_HEADERS})

# share headers with all iDynTree targets
set(libraryname idyntree-modelio-urdf)

add_library(${libraryname} ${IDYNTREE_MODELIO_URDF_SOURCES} ${IDYNTREE_MODELIO_URDF_HEADERS})

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>"
                                                  ${IDYNTREE_TREE_INCLUDE_DIRS}
                                          PRIVATE ${EIGEN3_INCLUDE_DIR}
                                                  ${TinyXML_INCLUDE_DIRS})

target_link_libraries(${libraryname} LINK_PUBLIC idyntree-model idyntree-sensors
                                     LINK_PRIVATE ${TinyXML_LIBRARIES})
                                     
if(NOT CMAKE_VERSION VERSION_LESS 2.8.12)
    target_compile_options(${libraryname} PRIVATE ${IDYNTREE_WARNING_FLAGS})
endif()

target_include_directories(${libraryname} PRIVATE ${EIGEN3_INCLUDE_DIR}
                                                  ${TinyXML_INCLUDE_DIRS})

# Ensure that build include directories are always included before system ones
get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)
target_include_directories(${libraryname} BEFORE PRIVATE ${IDYNTREE_TREE_INCLUDE_DIRS})

set_property(TARGET ${libraryname} PROPERTY PUBLIC_HEADER ${IDYNTREE_MODELIO_URDF_HEADERS})

install(TARGETS ${libraryname}
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/ModelIO)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})

get_property(IDYNTREE_COMP_INCLUDE_DIRS TARGET ${libraryname} PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS ${IDYNTREE_COMP_INCLUDE_DIRS})

if(IDYNTREE_COMPILE_TESTS)
    add_subdirectory(tests)
endif(IDYNTREE_COMPILE_TESTS)
