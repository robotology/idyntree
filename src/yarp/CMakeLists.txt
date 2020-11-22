# Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

project(iDynTree_YARP CXX)

SET(iDynTree_YARP_source src/YARPConversions.cpp
                         src/YARPConfigurationsLoader.cpp)

SET(iDynTree_YARP_header include/iDynTree/yarp/YARPConversions.h
                         include/iDynTree/yarp/YARPEigenConversions.h
                         include/iDynTree/yarp/YARPConfigurationsLoader.h)

SOURCE_GROUP("Source Files" FILES ${iDynTree_YARP_source})
SOURCE_GROUP("Header Files" FILES ${iDynTree_YARP_header})

include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/bin"
                          LIB_DIRS "${CMAKE_INSTALL_PREFIX}/lib"
                          DEPENDS IDYNTREE_ENABLE_RPATH
                          USE_LINK_PATH)


add_library(idyntree-yarp ${iDynTree_YARP_source} ${iDynTree_YARP_header})
add_library(iDynTree::idyntree-yarp ALIAS idyntree-yarp)

set_target_properties(idyntree-yarp PROPERTIES PUBLIC_HEADER "${iDynTree_YARP_header}")

target_include_directories(idyntree-yarp PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")

target_link_libraries(idyntree-yarp idyntree-core
                                    idyntree-model
                                    YARP::YARP_os
                                    YARP::YARP_math)
                                    
target_compile_options(idyntree-yarp PRIVATE ${IDYNTREE_WARNING_FLAGS})

install(TARGETS idyntree-yarp
        EXPORT iDynTree
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/yarp" COMPONENT dev)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS idyntree-yarp)
