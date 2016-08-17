# Copyright: (C) 2016 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

set(iDynTree_visualization_source src/Visualizer.cpp)
set(iDynTree_visualization_header include/iDynTree/Visualizer.h)

source_group("Source Files" FILES ${iDynTree_visualization_source})
source_group("Header Files" FILES ${iDynTree_visualization_header})

# share headers with all iDynTree targets
set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include")
get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)

include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/bin"
                          LIB_DIRS "${CMAKE_INSTALL_PREFIX}/lib"
                          DEPENDS IDYNTREE_ENABLE_RPATH
                          USE_LINK_PATH)


add_library(idyntree-visualization ${iDynTree_visualization_source} ${iDynTree_visualization_header})

set_target_properties(idyntree-visualization PROPERTIES PUBLIC_HEADER "${iDynTree_visualization_header}")

target_include_directories(idyntree-visualization PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include;${IDYNTREE_TREE_INCLUDE_DIRS}>"
                                                         "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>")

include_directories(${IDYNTREE_TREE_INCLUDE_DIRS})

target_link_libraries(idyntree-visualization LINK_PUBLIC idyntree-core
                                                         idyntree-model)

# To avoid having to regenerate the SWIG bindings code, we compile the visualization library
# even if irrlicht is not used, this can be removed when we stop to commit swig generated code
# to the repo 
if(IDYNTREE_USES_IRRLICHT)
    find_package(OpenGL)
    find_package(Irrlicht REQUIRED)
    target_include_directories(idyntree-visualization PRIVATE ${IRRLICHT_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR})
    target_link_libraries(idyntree-visualization LINK_PUBLIC ${IRRLICHT_LIBRARY} ${OPENGL_LIBRARIES})
    add_definitions(-DIDYNTREE_USES_IRRLICHT)
endif()

install(TARGETS idyntree-visualization
        EXPORT iDynTree
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/iDynTree" COMPONENT dev)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS idyntree-visualization)
