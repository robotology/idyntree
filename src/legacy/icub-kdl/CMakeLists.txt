# Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

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

# Legacy library, remove deprecated warnings when compiling
idyntree_disable_deprecation_warnings()

set(libraryname idyntree-icub-kdl)
add_library(${libraryname} ${iDynTree_ICUB_KDL_source} ${iDynTree_ICUB_KDL_header})

set_target_properties(${libraryname} PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                           SOVERSION ${${VARS_PREFIX}_VERSION}
                                           PUBLIC_HEADER "${iDynTree_ICUB_KDL_header}")

get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>")

target_link_libraries(${libraryname} LINK_PUBLIC  skinDynLib iKin iDyn
                                            ${GSL_LIBRARIES}
                                            ${YARP_LIBRARIES}
                                            idyntree-core
                                            idyntree-sensors
                                            idyntree-modelio-urdf-kdl
                                            idyntree-regressors
                                            idyntree-yarp-kdl
                                            ${Boost_LIBRARIES}
                                            ${TinyXML_LIBRARIES})

if(NOT CMAKE_VERSION VERSION_LESS 2.8.12)
    target_include_directories(${libraryname} SYSTEM INTERFACE ${skinDynLib_INCLUDE_DIRS} ${iDyn_INCLUDE_DIRS})
else()
    target_include_directories(${libraryname} INTERFACE ${skinDynLib_INCLUDE_DIRS} ${iDyn_INCLUDE_DIRS})
endif()

include_directories(SYSTEM ${skinDynLib_INCLUDE_DIRS}
                           ${iDyn_INCLUDE_DIRS}
                           ${GSL_INCLUDE_DIRS}
                           ${YARP_INCLUDE_DIRS}
                           ${EIGEN3_INCLUDE_DIR}
                           ${orocos_kdl_INCLUDE_DIRS})

# Ensure that build include directories are always included before system ones
get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)
target_include_directories(${libraryname} BEFORE PRIVATE ${IDYNTREE_TREE_INCLUDE_DIRS})

if((${CMAKE_CXX_COMPILER_ID} MATCHES "GNU") OR (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang"))
    target_compile_options(${libraryname} PRIVATE "-Wno-deprecated")
endif()

install(TARGETS idyntree-icub-kdl
        EXPORT iDynTree
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/iCub/iDynTree" COMPONENT dev)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS idyntree-icub-kdl)

set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)


if(IDYNTREE_COMPILE_TESTS)
    add_subdirectory(tests)
endif(IDYNTREE_COMPILE_TESTS)
