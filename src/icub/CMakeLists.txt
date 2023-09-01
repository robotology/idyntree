# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

SET(iDynTree_ICUB_header include/iDynTree/iKinConversions.h
                         include/iDynTree/iKinConversionsImplementation.h
                         include/iDynTree/skinDynLibConversions.h
                         include/iDynTree/skinDynLibConversionsImplementation.h)

SOURCE_GROUP("Header Files" FILES ${iDynTree_ICUB_header})

set(libraryname idyntree-icub)
add_library(${libraryname} INTERFACE)
add_library(iDynTree::${libraryname} ALIAS ${libraryname})

set_target_properties(${libraryname} PROPERTIES PUBLIC_HEADER "${iDynTree_ICUB_header}")

target_include_directories(${libraryname} INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")

target_link_libraries(${libraryname} INTERFACE idyntree-core idyntree-model idyntree-yarp idyntree-estimation)

install(TARGETS ${libraryname}
        EXPORT iDynTree
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/iDynTree" COMPONENT dev)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS idyntree-icub)

