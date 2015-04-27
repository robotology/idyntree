# Copyright: (C) 2015 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(iDynTree_Base CXX)

if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
endif(MSVC)

#local headers
include_directories(include/iDynTree/Core)

set(IDYNTREE_CORE_EXP_HEADERS include/iDynTree/Core/Position.h
                              include/iDynTree/Core/PositionRaw.h
                              include/iDynTree/Core/PositionSemantics.h
                              include/iDynTree/Core/Utils.h)

#file (GLOB IDYNTREE_BASE_PRIVATE_INCLUDES)

set(IDYNTREE_CORE_EXP_HEADERS src/Position.cpp
                              src/PositionRaw.cpp
                              src/PositionSemantics.cpp
                              src/Utils.cpp)

SOURCE_GROUP("Source Files" FILES ${IDYNTREE_CORE_EXP_SOURCES})
SOURCE_GROUP("Header Files" FILES ${IDYNTREE_CORE_EXP_HEADERS})
#SOURCE_GROUP("Header Files\\Private" FILES ${IDYNTREE_CORE_EXP_PRIVATE_INCLUDES})

# share headers with all iDynTree targes
set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include")

set(libraryname idyntree-core-exp)

add_library(${libraryname} ${IDYNTREE_CORE_EXP_SOURCES} ${IDYNTREE_CORE_EXP_HEADERS} ${IDYNTREE_CORE_EXP_PRIVATE_INCLUDES})

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
                                                "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>"
                                         PRIVATE ${EIGEN3_INCLUDE_DIR})

set_property(TARGET ${libraryname} PROPERTY PUBLIC_HEADER ${IDYNTREE_CORE_EXP_HEADERS})

install(TARGETS ${libraryname}
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/Core
        PRIVATE_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/Core/impl)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})
