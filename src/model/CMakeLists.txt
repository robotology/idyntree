# Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

project(iDynTree_Model CXX)

set(IDYNTREE_MODEL_HEADERS include/iDynTree/Model/ContactWrench.h
                           include/iDynTree/Model/DenavitHartenberg.h
                           include/iDynTree/Model/FixedJoint.h
                           include/iDynTree/Model/ForwardKinematics.h
                           include/iDynTree/Model/FreeFloatingState.h
                           include/iDynTree/Model/FreeFloatingMatrices.h
                           include/iDynTree/Model/IJoint.h
                           include/iDynTree/Model/Dynamics.h
                           include/iDynTree/Model/DynamicsLinearization.h
                           include/iDynTree/Model/DynamicsLinearizationHelpers.h
                           include/iDynTree/Model/Indeces.h
                           include/iDynTree/Model/Indices.h
                           include/iDynTree/Model/InertialParametersHelpers.h
                           include/iDynTree/Model/Jacobians.h
                           include/iDynTree/Model/JointState.h
                           include/iDynTree/Model/LinkTraversalsCache.h
                           include/iDynTree/Model/Link.h
                           include/iDynTree/Model/LinkState.h
                           include/iDynTree/Model/Model.h
                           include/iDynTree/Model/ModelTransformers.h
                           include/iDynTree/Model/MovableJointImpl.h
                           include/iDynTree/Model/RevoluteJoint.h
                           include/iDynTree/Model/PrismaticJoint.h
                           include/iDynTree/Model/SolidShapes.h
                           include/iDynTree/Model/SubModel.h
                           include/iDynTree/Model/Traversal.h)

set(IDYNTREE_MODEL_PRIVATE_INCLUDES include/iDynTree/Model/ModelTestUtils.h)

set(IDYNTREE_MODEL_SOURCES src/ContactWrench.cpp
                           src/DenavitHartenberg.cpp
                           src/FixedJoint.cpp
                           src/ForwardKinematics.cpp
                           src/FreeFloatingState.cpp
                           src/FreeFloatingMatrices.cpp
                           src/Indices.cpp
                           src/InertialParametersHelpers.cpp
                           src/Dynamics.cpp
                           src/DynamicsLinearization.cpp
                           src/DynamicsLinearizationHelpers.cpp
                           src/Link.cpp
                           src/LinkState.cpp
                           src/LinkTraversalsCache.cpp
                           src/Jacobians.cpp
                           src/JointState.cpp
                           src/Model.cpp
                           src/ModelTransformers.cpp
                           src/ModelInterfaceDestructors.cpp
                           src/RevoluteJoint.cpp
                           src/PrismaticJoint.cpp
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

target_link_libraries(${libraryname} PUBLIC idyntree-core)

# Depending on the IDYNTREE_USES_ASSIMP option, some function are implemented or not
# Note that this flag needs to be considered only in .cpp files, otherwise it could
# create ABI inconsistencies as it is not defined for the downstream compilation units
if (IDYNTREE_USES_ASSIMP)
  target_compile_definitions(${libraryname} PRIVATE IDYNTREE_USES_ASSIMP)
  target_include_directories(${libraryname} PRIVATE "${ASSIMP_INCLUDE_DIRS}")
  link_directories("${ASSIMP_LIBRARY_DIRS}")
  target_link_libraries(${libraryname} PRIVATE "${ASSIMP_LIBRARIES}")
endif()

# Ensure that build include directories are always included before system ones
get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)
target_include_directories(${libraryname} BEFORE PRIVATE ${IDYNTREE_TREE_INCLUDE_DIRS})

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
