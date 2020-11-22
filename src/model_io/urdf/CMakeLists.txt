# Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

project(iDynTree_ModelIO_URDF CXX)

set(IDYNTREE_MODELIO_URDF_HEADERS include/iDynTree/ModelIO/URDFDofsImport.h
                                  include/iDynTree/ModelIO/ModelLoader.h
                                  include/iDynTree/ModelIO/ModelExporter.h
                                  include/iDynTree/ModelIO/ModelCalibrationHelper.h)

set(IDYNTREE_MODELIO_URDF_PRIVATE_HEADERS include/private/URDFDocument.h
                                          include/private/InertialElement.h
                                          include/private/JointElement.h
                                          include/private/LinkElement.h
                                          include/private/RobotElement.h
                                          include/private/OriginElement.h
                                          include/private/SensorElement.h
                                          include/private/ForceTorqueSensorElement.h
                                          include/private/MaterialElement.h
                                          include/private/VisualElement.h
                                          include/private/GeometryElement.h
                                          include/private/URDFParsingUtils.h
                                          include/private/URDFModelExport.h)

set(IDYNTREE_MODELIO_URDF_XMLELEMENTS_SOURCES src/URDFDocument.cpp
                                              src/InertialElement.cpp
                                              src/JointElement.cpp
                                              src/LinkElement.cpp
                                              src/RobotElement.cpp
                                              src/OriginElement.cpp
                                              src/SensorElement.cpp
                                              src/ForceTorqueSensorElement.cpp
                                              src/MaterialElement.cpp
                                              src/VisualElement.cpp
                                              src/GeometryElement.cpp)

set(IDYNTREE_MODELIO_URDF_SOURCES src/URDFDofsImport.cpp
                                  src/ModelLoader.cpp
                                  src/ModelExporter.cpp
                                  src/ModelCalibrationHelper.cpp
                                  src/URDFModelExport.cpp)

SOURCE_GROUP("Source Files" FILES ${IDYNTREE_MODELIO_URDF_SOURCES})
SOURCE_GROUP("Source Files\\XML Elements" FILES ${IDYNTREE_MODELIO_URDF_XMLELEMENTS_SOURCES})
SOURCE_GROUP("Header Files" FILES ${IDYNTREE_MODELIO_URDF_HEADERS})
SOURCE_GROUP("Private Header Files" FILES ${IDYNTREE_MODELIO_URDF_PRIVATE_HEADERS})

list(APPEND IDYNTREE_MODELIO_URDF_SOURCES ${IDYNTREE_MODELIO_URDF_XMLELEMENTS_SOURCES})

# share headers with all iDynTree targets
set(libraryname idyntree-modelio-urdf)

add_library(${libraryname} ${IDYNTREE_MODELIO_URDF_SOURCES} ${IDYNTREE_MODELIO_URDF_HEADERS} ${IDYNTREE_MODELIO_URDF_PRIVATE_HEADERS} $<TARGET_OBJECTS:idyntree-private-fpconv>)
add_library(iDynTree::${libraryname} ALIAS ${libraryname})

target_compile_features(${libraryname} PRIVATE cxx_auto_type cxx_delegating_constructors cxx_final cxx_lambdas cxx_lambda_init_captures)

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/private>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")

target_link_libraries(${libraryname} PUBLIC idyntree-core idyntree-model idyntree-sensors idyntree-modelio-xml
                                     PRIVATE Eigen3::Eigen LibXml2::LibXml2)
# See https://stackoverflow.com/questions/38832528/transitive-target-include-directories-on-object-libraries
# Can be removed with CMake 3.12
target_include_directories(${libraryname} PRIVATE $<TARGET_PROPERTY:idyntree-private-fpconv,INTERFACE_INCLUDE_DIRECTORIES>)

target_compile_options(${libraryname} PRIVATE ${IDYNTREE_WARNING_FLAGS})

set_property(TARGET ${libraryname} PROPERTY PUBLIC_HEADER ${IDYNTREE_MODELIO_URDF_HEADERS})

install(TARGETS ${libraryname}
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/ModelIO)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})

if(IDYNTREE_COMPILE_TESTS)
    add_subdirectory(tests)
endif(IDYNTREE_COMPILE_TESTS)
