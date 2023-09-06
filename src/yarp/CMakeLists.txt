# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

SET(iDynTree_YARP_header include/iDynTree/YARPConversions.h
                         include/iDynTree/YARPEigenConversions.h
                         include/iDynTree/YARPConfigurationsLoader.h
                         include/iDynTree/YARPConversionsImplementation.h
                         include/iDynTree/YARPConfigurationsLoaderImplementation.h)

SOURCE_GROUP("Header Files" FILES ${iDynTree_YARP_header})

add_library(idyntree-yarp INTERFACE)
add_library(iDynTree::idyntree-yarp ALIAS idyntree-yarp)

set_target_properties(idyntree-yarp PROPERTIES PUBLIC_HEADER "${iDynTree_YARP_header}")

target_include_directories(idyntree-yarp INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                   "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")

target_link_libraries(idyntree-yarp INTERFACE idyntree-core idyntree-model)

install(TARGETS idyntree-yarp
        EXPORT iDynTree
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/iDynTree" COMPONENT dev)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS idyntree-yarp)

# Install deprecated headers
install(DIRECTORY include/iDynTree/yarp
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree)
