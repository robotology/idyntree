# Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

SET(iDynTree_ICUB_source src/iKinConversions.cpp
                         src/skinDynLibConversions.cpp)

SET(iDynTree_ICUB_header include/iDynTree/iKinConversions.h
                         include/iDynTree/skinDynLibConversions.h)

SOURCE_GROUP("Source Files" FILES ${iDynTree_ICUB_source})
SOURCE_GROUP("Header Files" FILES ${iDynTree_ICUB_header})

include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/bin"
                          LIB_DIRS "${CMAKE_INSTALL_PREFIX}/lib"
                          DEPENDS IDYNTREE_ENABLE_RPATH
                          USE_LINK_PATH)

set(libraryname idyntree-icub)
add_library(${libraryname} ${iDynTree_ICUB_source} ${iDynTree_ICUB_header})
add_library(iDynTree::${libraryname} ALIAS ${libraryname})

set_target_properties(${libraryname} PROPERTIES PUBLIC_HEADER "${iDynTree_ICUB_header}")


target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")

target_link_libraries(${libraryname} LINK_PUBLIC iKin skinDynLib idyntree-core idyntree-model idyntree-yarp idyntree-estimation)

target_compile_options(${libraryname} PRIVATE ${IDYNTREE_WARNING_FLAGS})

target_include_directories(${libraryname} PRIVATE SYSTEM ${skinDynLib_INCLUDE_DIRS})
target_include_directories(${libraryname} PRIVATE SYSTEM ${iKin_INCLUDE_DIRS})


install(TARGETS ${libraryname}
        EXPORT iDynTree
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/iDynTree" COMPONENT dev)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS idyntree-icub)

