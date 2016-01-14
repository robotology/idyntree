# Copyright: (C) 2015 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(iDynTree_YARP_KDL CXX)

SET(iDynTree_YARP_KDL_source src/DynTree.cpp
                         src/yarp_kdl.cpp
                         src/yarp_iDynTree.cpp)

SET(iDynTree_YARP_KDL_header include/iCub/iDynTree/DynTree.h
                  include/iCub/iDynTree/yarp_kdl.h
                  include/iCub/iDynTree/yarp_iDynTree.h)

SOURCE_GROUP("Source Files" FILES ${iDynTree_YARP_KDL_source})
SOURCE_GROUP("Header Files" FILES ${iDynTree_YARP_KDL_header})

# share headers with all iDynTree targes
set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include")
get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)

include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/bin"
                          LIB_DIRS "${CMAKE_INSTALL_PREFIX}/lib"
                          DEPENDS IDYNTREE_ENABLE_RPATH
                          USE_LINK_PATH)


add_library(idyntree-yarp-kdl ${iDynTree_YARP_KDL_source} ${iDynTree_YARP_KDL_header})

set_target_properties(idyntree-yarp-kdl PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                           SOVERSION ${${VARS_PREFIX}_VERSION}
                                           PUBLIC_HEADER "${iDynTree_YARP_KDL_header}")


target_include_directories(idyntree-yarp-kdl PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include;${IDYNTREE_TREE_INCLUDE_DIRS}>"
                                                    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>")

include_directories(${IDYNTREE_TREE_INCLUDE_DIRS})

target_include_directories(idyntree-yarp-kdl INTERFACE ${EIGEN3_INCLUDE_DIR}
                                                    ${YARP_INCLUDE_DIRS})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR}
                           ${YARP_INCLUDE_DIRS})

target_link_libraries(idyntree-yarp-kdl LINK_PUBLIC
                                            ${GSL_LIBRARIES}
                                            ${YARP_LIBRARIES}
                                            idyntree-core
                                            idyntree-sensors
                                            idyntree-modelio-urdf
                                            idyntree-regressors
                                            ${Boost_LIBRARIES}
                                            ${TinyXML_LIBRARIES})


install(TARGETS idyntree-yarp-kdl
        EXPORT iDynTree
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/iCub/iDynTree" COMPONENT dev)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS idyntree-yarp-kdl)
