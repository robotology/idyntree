# Copyright: (C) 2015 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(iDynTree_ICUB_KDL CXX)

SET(iDynTree_ICUB_KDL_source src/TorqueEstimationTree.cpp
                         src/iDyn2KDL.cpp
                         src/idyn2kdl_icub.cpp)

SET(iDynTree_ICUB_KDL_header
                  include/iCub/iDynTree/iDyn2KDL.h
                  include/iCub/iDynTree/TorqueEstimationTree.h
                  include/iCub/iDynTree/idyn2kdl_icub.h)

SOURCE_GROUP("Source Files" FILES ${iDynTree_ICUB_KDL_source})
SOURCE_GROUP("Header Files" FILES ${iDynTree_ICUB_KDL_header})

include_directories()

include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/bin"
                          LIB_DIRS "${CMAKE_INSTALL_PREFIX}/lib"
                          DEPENDS IDYNTREE_ENABLE_RPATH
                          USE_LINK_PATH)


add_library(idyntree-icub-kdl ${iDynTree_ICUB_KDL_source} ${iDynTree_ICUB_KDL_header})

set_target_properties(idyntree-icub-kdl PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                           SOVERSION ${${VARS_PREFIX}_VERSION}
                                           PUBLIC_HEADER "${iDynTree_ICUB_KDL_header}")

get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)

target_include_directories(idyntree-icub-kdl PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include;${IDYNTREE_TREE_INCLUDE_DIRS}>"
                                                "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>")

target_link_libraries(idyntree-icub-kdl LINK_PUBLIC  skinDynLib iKin iDyn
                                            ${GSL_LIBRARIES}
                                            ${YARP_LIBRARIES}
                                            idyntree-core
                                            idyntree-sensors
                                            idyntree-modelio-urdf-kdl
                                            idyntree-regressors
                                            idyntree-yarp-kdl
                                            ${Boost_LIBRARIES}
                                            ${TinyXML_LIBRARIES})

target_include_directories(idyntree-icub-kdl INTERFACE ${skinDynLib_INCLUDE_DIRS} ${iDyn_INCLUDE_DIRS})

include_directories(SYSTEM ${skinDynLib_INCLUDE_DIRS}
                           ${iDyn_INCLUDE_DIRS}
                           ${GSL_INCLUDE_DIRS}
                           ${YARP_INCLUDE_DIRS}
                           ${EIGEN3_INCLUDE_DIR}
                           ${orocos_kdl_INCLUDE_DIRS})

install(TARGETS idyntree-icub-kdl
        EXPORT iDynTree
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/iCub/iDynTree" COMPONENT dev)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS idyntree-icub-kdl)

if(IDYNTREE_COMPILE_TESTS)
    add_subdirectory(tests)
endif(IDYNTREE_COMPILE_TESTS)
