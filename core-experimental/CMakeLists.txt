# Copyright: (C) 2015 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(iDynTree_Base CXX)

#local headers
include_directories(include/iDynTree/Core)

set(IDYNTREE_CORE_EXP_HEADERS include/iDynTree/Core/IMatrix.h
                              include/iDynTree/Core/IVector.h
                              include/iDynTree/Core/MatrixDynSize.h
                              include/iDynTree/Core/Position.h
                              include/iDynTree/Core/PositionRaw.h
                              include/iDynTree/Core/PositionSemantics.h
                              include/iDynTree/Core/Rotation.h
                              include/iDynTree/Core/RotationRaw.h
                              include/iDynTree/Core/RotationSemantics.h
                              include/iDynTree/Core/RotationalInertiaRaw.h
                              include/iDynTree/Core/SpatialAcc.h
                              include/iDynTree/Core/SpatialForceVectorRaw.h
                              include/iDynTree/Core/SpatialInertiaRaw.h
                              include/iDynTree/Core/SpatialInertia.h
                              include/iDynTree/Core/SpatialMomentum.h
                              include/iDynTree/Core/SpatialMotionVectorRaw.h
                              include/iDynTree/Core/Transform.h
                              include/iDynTree/Core/TransformSemantics.h
                              include/iDynTree/Core/Twist.h
                              include/iDynTree/Core/Utils.h
                              include/iDynTree/Core/Vector6.h
                              include/iDynTree/Core/VectorDynSize.h
                              include/iDynTree/Core/Wrench.h)

set(IDYNTREE_CORE_EXP_PRIVATE_INCLUDES include/iDynTree/Core/PrivateUtils.h)


set(IDYNTREE_CORE_EXP_SOURCES src/InterfaceDestructors.cpp
                              src/MatrixDynSize.cpp
                              src/Position.cpp
                              src/PositionRaw.cpp
                              src/PositionSemantics.cpp
                              src/Rotation.cpp
                              src/RotationRaw.cpp
                              src/RotationSemantics.cpp
                              src/RotationalInertiaRaw.cpp
                              src/SpatialAcc.cpp
                              src/SpatialForceVectorRaw.cpp
                              src/SpatialMomentum.cpp
                              src/SpatialMotionVectorRaw.cpp
                              src/SpatialInertiaRaw.cpp
                              src/SpatialInertia.cpp
                              src/Transform.cpp
                              src/TransformSemantics.cpp
                              src/Twist.cpp
                              src/Utils.cpp
                              src/Vector6.cpp
                              src/VectorDynSize.cpp
                              src/Wrench.cpp
                              src/PrivateUtils.cpp)

SOURCE_GROUP("Source Files" FILES ${IDYNTREE_CORE_EXP_SOURCES})
SOURCE_GROUP("Header Files" FILES ${IDYNTREE_CORE_EXP_HEADERS})
#SOURCE_GROUP("Header Files\\Private" FILES ${IDYNTREE_CORE_EXP_PRIVATE_INCLUDES})

# share headers with all iDynTree targes
set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include")

set(libraryname idyntree-core-exp)

add_library(${libraryname} ${IDYNTREE_CORE_EXP_SOURCES} ${IDYNTREE_CORE_EXP_HEADERS} ${IDYNTREE_CORE_EXP_PRIVATE_INCLUDES})

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>"
                                          PRIVATE ${EIGEN3_INCLUDE_DIR})

set_property(TARGET ${libraryname} PROPERTY PUBLIC_HEADER ${IDYNTREE_CORE_EXP_HEADERS})

install(TARGETS ${libraryname}
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/Core
        PRIVATE_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/Core/impl)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})

if(IDYNTREE_COMPILE_TESTS)
    add_subdirectory(tests)
endif(IDYNTREE_COMPILE_TESTS)