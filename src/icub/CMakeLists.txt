# Copyright: (C) 2015 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(iDynTree_ICUB CXX)

SET(iDynTree_ICUB_source
                         src/TorqueEstimationTree.cpp
                         src/iDyn2KDL.cpp
                         src/idyn2kdl_icub.cpp)

SET(iDynTree_ICUB_header
                  include/iCub/iDynTree/iDyn2KDL.h
                  include/iCub/iDynTree/TorqueEstimationTree.h
                  include/iCub/iDynTree/idyn2kdl_icub.h)

SOURCE_GROUP("Source Files" FILES ${iDynTree_ICUB_source})
SOURCE_GROUP("Header Files" FILES ${iDynTree_ICUB_header})

include_directories()

include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/bin"
                          LIB_DIRS "${CMAKE_INSTALL_PREFIX}/lib"
                          DEPENDS IDYNTREE_ENABLE_RPATH
                          USE_LINK_PATH)


add_library(idyntree-icub ${iDynTree_ICUB_source} ${iDynTree_ICUB_header})

set_target_properties(idyntree-icub PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                           SOVERSION ${${VARS_PREFIX}_VERSION}
                                           PUBLIC_HEADER "${iDynTree_ICUB_header}")

target_include_directories(idyntree-icub PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>"
                                                ${skinDynLib_INCLUDE_DIRS}
                                                ${iDyn_INCLUDE_DIRS}
                                                ${GSL_INCLUDE_DIRS}
                                                ${YARP_INCLUDE_DIRS}
                                                ${EIGEN3_INCLUDE_DIR}
                                                ${orocos_kdl_INCLUDE_DIRS})


target_link_libraries(idyntree-icub LINK_PUBLIC  skinDynLib iKin iDyn
                                            ${GSL_LIBRARIES}
                                            ${YARP_LIBRARIES}
                                            idyntree-core
                                            idyntree-sensors
                                            idyntree-modelio-urdf
                                            idyntree-regressors
                                            idyntree-yarp
                                            ${Boost_LIBRARIES}
                                            ${TinyXML_LIBRARIES})


install(TARGETS idyntree-icub
        EXPORT iDynTree
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/iCub/iDynTree" COMPONENT dev)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS idyntree-icub)

if(IDYNTREE_COMPILE_TESTS)
    add_subdirectory(tests)
endif(IDYNTREE_COMPILE_TESTS)
