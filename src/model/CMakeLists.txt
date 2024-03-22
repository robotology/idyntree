# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


set(IDYNTREE_MODEL_HEADERS include/iDynTree/ContactWrench.h
                           include/iDynTree/DenavitHartenberg.h
                           include/iDynTree/FixedJoint.h
                           include/iDynTree/ForwardKinematics.h
                           include/iDynTree/FreeFloatingState.h
                           include/iDynTree/FreeFloatingMatrices.h
                           include/iDynTree/IJoint.h
                           include/iDynTree/Dynamics.h
                           include/iDynTree/DynamicsLinearization.h
                           include/iDynTree/DynamicsLinearizationHelpers.h
                           include/iDynTree/Indices.h
                           include/iDynTree/Jacobians.h
                           include/iDynTree/JointState.h
                           include/iDynTree/LinkTraversalsCache.h
                           include/iDynTree/Link.h
                           include/iDynTree/LinkState.h
                           include/iDynTree/Model.h
                           include/iDynTree/ModelTransformers.h
                           include/iDynTree/MovableJointImpl.h
                           include/iDynTree/RevoluteJoint.h
                           include/iDynTree/PrismaticJoint.h
                           include/iDynTree/SolidShapes.h
                           include/iDynTree/SubModel.h
                           include/iDynTree/Traversal.h
                           include/iDynTree/ModelTestUtils.h
                           include/iDynTree/AllSensorsTypes.h
                           include/iDynTree/Sensors.h
                           include/iDynTree/SixAxisForceTorqueSensor.h
                           include/iDynTree/GyroscopeSensor.h
                           include/iDynTree/AccelerometerSensor.h
                           include/iDynTree/ThreeAxisAngularAccelerometerSensor.h
                           include/iDynTree/ThreeAxisForceTorqueContactSensor.h
                           include/iDynTree/PredictSensorsMeasurements.h
                           include/iDynTree/ModelSensorsTransformers.h)

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
                           src/Traversal.cpp
                           src/Sensors.cpp
                           src/SixAxisForceTorqueSensor.cpp
                           src/AccelerometerSensor.cpp
                           src/GyroscopeSensor.cpp
                           src/ThreeAxisAngularAccelerometerSensor.cpp
                           src/ThreeAxisForceTorqueContactSensor.cpp
                           src/PredictSensorsMeasurements.cpp
                           src/ModelSensorsTransformers.cpp)

SOURCE_GROUP("Source Files" FILES ${IDYNTREE_MODEL_SOURCES})
SOURCE_GROUP("Header Files" FILES ${IDYNTREE_MODEL_HEADERS})

set(libraryname idyntree-model)

add_library(${libraryname} ${IDYNTREE_MODEL_SOURCES} ${IDYNTREE_MODEL_HEADERS})
add_library(iDynTree::${libraryname} ALIAS ${libraryname})

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
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree
        PRIVATE_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/impl)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})

if(BUILD_TESTING)
    add_subdirectory(tests)
endif(BUILD_TESTING)

# Add deprecated target iDynTree::idyntree-sensors
add_library(idyntree-sensors INTERFACE)
add_library(iDynTree::idyntree-sensors ALIAS idyntree-sensors)
set_target_properties(idyntree-sensors PROPERTIES INTERFACE_LINK_LIBRARIES iDynTree::idyntree-model)
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.17)
    set_target_properties(idyntree-sensors PROPERTIES DEPRECATION "Linking the iDynTree::idyntree-sensors target is deprecated, please link iDynTree::idyntree-model that provides the symbols formerly containd in iDynTree::idyntree-sensors")
endif()
set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS idyntree-sensors)
install(TARGETS idyntree-sensors
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib)

# Install deprecated headers
install(DIRECTORY include/iDynTree/Model
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree)
install(DIRECTORY include/iDynTree/Sensors
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree)