# Copyright: (C) 2016 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

set(iDynTree_visualization_source src/Visualizer.cpp)
set(iDynTree_visualization_header include/iDynTree/Visualizer.h)
set(iDynTree_visualization_private_headers)
set(iDynTree_visualization_private_source)


if(IDYNTREE_USES_IRRLICHT)
    # Some files are compiled only when irrlicht is available
    set(iDynTree_visualization_private_headers src/IrrlichtUtils.h
                                               src/Camera.h
                                               src/Environment.h
                                               src/ModelVisualization.h
                                               src/JetsVisualization.h
                                               src/VectorsVisualization.h
                                               src/Light.h
                                               src/FloorGridSceneNode.h)
    set(iDynTree_visualization_private_source  src/Camera.cpp
                                               src/Environment.cpp
                                               src/ModelVisualization.cpp
                                               src/JetsVisualization.cpp
                                               src/VectorsVisualization.cpp
                                               src/Light.cpp
                                               src/FloorGridSceneNode.cpp)
endif()

source_group("Source Files" FILES ${iDynTree_visualization_source})
source_group("Header Files" FILES ${iDynTree_visualization_header})
source_group("Private Source Files" FILES ${iDynTree_visualization_source})
source_group("Private Header Files" FILES ${iDynTree_visualization_header})

include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/bin"
                          LIB_DIRS "${CMAKE_INSTALL_PREFIX}/lib"
                          DEPENDS IDYNTREE_ENABLE_RPATH
                          USE_LINK_PATH)


set(libraryname idyntree-visualization)
add_library(${libraryname} ${iDynTree_visualization_source} ${iDynTree_visualization_header}
                           ${iDynTree_visualization_private_source} ${iDynTree_visualization_private_headers})
add_library(iDynTree::${libraryname} ALIAS ${libraryname})

set_target_properties(${libraryname} PROPERTIES PUBLIC_HEADER "${iDynTree_visualization_header}")

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")

target_link_libraries(${libraryname} PUBLIC idyntree-core
                                            idyntree-model
                                     PRIVATE Eigen3::Eigen)

# enable warnings on this part of the add_library
target_compile_options(${libraryname} PRIVATE ${IDYNTREE_WARNING_FLAGS})

# To avoid having to regenerate the SWIG bindings code, we compile the visualization library
# even if irrlicht is not used, this can be removed when we stop to commit swig generated code
# to the repo
if(IDYNTREE_USES_IRRLICHT)
    find_package(OpenGL)
    find_package(Irrlicht REQUIRED)

    target_include_directories(${libraryname} PRIVATE SYSTEM ${IRRLICHT_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR})
    target_link_libraries(${libraryname} LINK_PRIVATE ${IRRLICHT_LIBRARY} ${OPENGL_LIBRARIES})
    if(MSVC)
       add_definitions(-D_USE_MATH_DEFINES)
    endif()
    add_definitions(-DIDYNTREE_USES_IRRLICHT)

    # On Apple system, irrlicht uses some system libraries
    if (APPLE)
    	find_library(CARBON_LIBRARY Carbon)
    	find_library(COCOA_LIBRARY Cocoa)
    	find_library(IOKIT_LIBRARY IOKit)
    	target_link_libraries(${libraryname} LINK_PRIVATE ${CARBON_LIBRARY} ${COCOA_LIBRARY} ${IOKIT_LIBRARY})
    endif ()
endif()

install(TARGETS ${libraryname}
        EXPORT iDynTree
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/iDynTree" COMPONENT dev)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})

if(IDYNTREE_COMPILE_TESTS AND IDYNTREE_USES_IRRLICHT)
    add_subdirectory(tests)
endif()
