# Copyright: (C) 2015 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(iDynTree_Model CXX)

set(IDYNTREE_MODEL_HEADERS include/iDynTree/Model/ContactWrench.h
                           include/iDynTree/Model/FixedJoint.h
                           include/iDynTree/Model/ForwardKinematics.h
                           include/iDynTree/Model/FreeFloatingState.h
                           include/iDynTree/Model/FreeFloatingMatrices.h
                           include/iDynTree/Model/IJoint.h
                           include/iDynTree/Model/Dynamics.h
                           include/iDynTree/Model/DynamicsLinearization.h
                           include/iDynTree/Model/DynamicsLinearizationHelpers.h
                           include/iDynTree/Model/Indeces.h
                           include/iDynTree/Model/Jacobians.h
                           include/iDynTree/Model/JointState.h
                           include/iDynTree/Model/Link.h
                           include/iDynTree/Model/LinkState.h
                           include/iDynTree/Model/Model.h
                           include/iDynTree/Model/ModelTransformers.h
                           include/iDynTree/Model/MovableJointImpl.h
                           include/iDynTree/Model/RevoluteJoint.h
                           include/iDynTree/Model/SolidShapes.h
                           include/iDynTree/Model/SubModel.h
                           include/iDynTree/Model/Traversal.h)

set(IDYNTREE_MODEL_PRIVATE_INCLUDES include/iDynTree/Model/ModelTestUtils.h)

set(IDYNTREE_MODEL_SOURCES src/ContactWrench.cpp
                           src/FixedJoint.cpp
                           src/ForwardKinematics.cpp
                           src/FreeFloatingState.cpp
                           src/FreeFloatingMatrices.cpp
                           src/Indeces.cpp
                           src/Dynamics.cpp
                           src/DynamicsLinearization.cpp
                           src/DynamicsLinearizationHelpers.cpp
                           src/Link.cpp
                           src/LinkState.cpp
                           src/Jacobians.cpp
                           src/JointState.cpp
                           src/Model.cpp
                           src/ModelTransformers.cpp
                           src/ModelInterfaceDestructors.cpp
                           src/RevoluteJoint.cpp
                           src/SolidShapes.cpp
                           src/SubModel.cpp
                           src/Traversal.cpp)

SOURCE_GROUP("Source Files" FILES ${IDYNTREE_MODEL_SOURCES})
SOURCE_GROUP("Header Files" FILES ${IDYNTREE_MODEL_HEADERS})
SOURCE_GROUP("Header Files\\Private" FILES ${IDYNTREE_MODEL_PRIVATE_INCLUDES})

set(libraryname idyntree-model)

add_library(${libraryname} ${IDYNTREE_MODEL_SOURCES} ${IDYNTREE_MODEL_HEADERS} ${IDYNTREE_MODEL_PRIVATE_INCLUDES})

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>"
                                          PRIVATE ${EIGEN3_INCLUDE_DIR})

target_link_libraries(${libraryname} idyntree-core)

set_property(TARGET ${libraryname} PROPERTY PUBLIC_HEADER ${IDYNTREE_MODEL_HEADERS})

install(TARGETS ${libraryname}
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/Model
        PRIVATE_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/Model/impl)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})

get_property(IDYNTREE_COMP_INCLUDE_DIRS TARGET ${libraryname} PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS ${IDYNTREE_COMP_INCLUDE_DIRS})


if(IDYNTREE_COMPILE_TESTS)
    add_subdirectory(tests)
endif(IDYNTREE_COMPILE_TESTS)
