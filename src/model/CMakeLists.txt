# Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

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
                           include/iDynTree/Model/Indices.h
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
                           include/iDynTree/Model/Traversal.h
                           include/iDynTree/Model/ModelTestUtils.h)

set(IDYNTREE_MODEL_SOURCES src/ContactWrench.cpp
                           src/DenavitHartenberg.cpp
                           src/FixedJoint.cpp
                           src/ForwardKinematics.cpp
                           src/FreeFloatingState.cpp
                           src/FreeFloatingMatrices.cpp
                           src/Indices.cpp
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

set(libraryname idyntree-model)

add_library(${libraryname} ${IDYNTREE_MODEL_SOURCES} ${IDYNTREE_MODEL_HEADERS})

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")

target_link_libraries(${libraryname} PUBLIC idyntree-core
                                     PRIVATE Eigen3::Eigen)

# On Windows we need to correctly export global constants that are not inlined with the use of GenerateExportHeader
# vtk 6.3 installs a GenerateExportHeader CMake module that shadows the official CMake module if find_package(VTK)
# is invoked, the issue has been solved in vtk >= 7.1, see https://gitlab.kitware.com/vtk/vtk/-/merge_requests/1593 .
# To ensure that the build works even if find_package(VTK) is invoked and VTK 6 is found, we manually include exactly the
# GenerateExportHeader that is shipped with CMake, extracting the CMake install prefix from the CMAKE_COMMAND variable
# This workaround can be removed once we will not support anymore platforms that ship with vtk 6.3 as main vtk version (Ubuntu 18.04).
get_filename_component(CMAKE_UPSTREAM_MODULE_LOCATION ${CMAKE_COMMAND} PATH)
get_filename_component(CMAKE_UPSTREAM_MODULE_LOCATION ${CMAKE_UPSTREAM_MODULE_LOCATION} PATH)
set(CMAKE_UPSTREAM_MODULE_LOCATION ${CMAKE_UPSTREAM_MODULE_LOCATION}/share/cmake-${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}/Modules)
if(EXISTS ${CMAKE_UPSTREAM_MODULE_LOCATION}/GenerateExportHeader.cmake)
    include(${CMAKE_UPSTREAM_MODULE_LOCATION}/GenerateExportHeader.cmake)
else()
    include(GenerateExportHeader)
endif()
generate_export_header(${libraryname} EXPORT_FILE_NAME ModelExport.h)
list(APPEND IDYNTREE_MODEL_HEADERS ${CMAKE_CURRENT_BINARY_DIR}/ModelExport.h)

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>")

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


if(IDYNTREE_COMPILE_TESTS)
    add_subdirectory(tests)
endif(IDYNTREE_COMPILE_TESTS)
