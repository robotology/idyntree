# Copyright (C) Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

set(IDYNTREE_CORE_EXP_HEADERS include/iDynTree/Core/Axis.h
                              include/iDynTree/Core/ArticulatedBodyInertia.h
                              include/iDynTree/Core/ClassicalAcc.h
                              include/iDynTree/Core/Direction.h
                              include/iDynTree/Core/EigenSparseHelpers.h
                              include/iDynTree/Core/EigenMathHelpers.h
                              include/iDynTree/Core/EigenHelpers.h
                              include/iDynTree/Core/InertiaNonLinearParametrization.h
                              include/iDynTree/Core/MatrixDynSize.h
                              include/iDynTree/Core/MatrixFixSize.h
                              include/iDynTree/Core/Position.h
                              include/iDynTree/Core/PositionRaw.h
                              include/iDynTree/Core/Rotation.h
                              include/iDynTree/Core/RotationRaw.h
                              include/iDynTree/Core/RotationalInertiaRaw.h
                              include/iDynTree/Core/SpatialAcc.h
                              include/iDynTree/Core/SpatialForceVector.h
                              include/iDynTree/Core/SpatialInertiaRaw.h
                              include/iDynTree/Core/SpatialInertia.h
                              include/iDynTree/Core/SpatialMomentum.h
                              include/iDynTree/Core/SpatialMotionVector.h
                              include/iDynTree/Core/TestUtils.h
                              include/iDynTree/Core/Transform.h
                              include/iDynTree/Core/TransformDerivative.h
                              include/iDynTree/Core/Twist.h
                              include/iDynTree/Core/Utils.h
                              include/iDynTree/Core/VectorFixSize.h
                              include/iDynTree/Core/VectorDynSize.h
                              include/iDynTree/Core/Wrench.h
                              include/iDynTree/Core/PrivateUtils.h
                              include/iDynTree/Core/PrivatePreProcessorUtils.h
                              include/iDynTree/Core/GeomVector3.h
                              include/iDynTree/Core/SpatialVector.h
                              include/iDynTree/Core/SparseMatrix.h
                              include/iDynTree/Core/Triplets.h
                              include/iDynTree/Core/CubicSpline.h
                              include/iDynTree/Core/Span.h
                              include/iDynTree/Core/SO3Utils.h
                              include/iDynTree/Core/MatrixView.h
                              # Deprecated headers
                              include/iDynTree/Core/AngularForceVector3.h
                              include/iDynTree/Core/AngularMotionVector3.h
                              include/iDynTree/Core/ForceVector3.h
                              include/iDynTree/Core/LinearForceVector3.h
                              include/iDynTree/Core/LinearMotionVector3.h
                              include/iDynTree/Core/MotionVector3.h)


set(IDYNTREE_CORE_EXP_SOURCES src/Axis.cpp
                              src/ArticulatedBodyInertia.cpp
                              src/ClassicalAcc.cpp
                              src/Direction.cpp
                              src/InertiaNonLinearParametrization.cpp
                              src/MatrixDynSize.cpp
                              src/GeomVector3.cpp
                              src/Position.cpp
                              src/PositionRaw.cpp
                              src/Rotation.cpp
                              src/RotationRaw.cpp
                              src/RotationalInertiaRaw.cpp
                              src/SpatialAcc.cpp
                              src/SpatialForceVector.cpp
                              src/SpatialMomentum.cpp
                              src/SpatialMotionVector.cpp
                              src/SpatialInertiaRaw.cpp
                              src/SpatialInertia.cpp
                              src/TestUtils.cpp
                              src/Transform.cpp
                              src/TransformDerivative.cpp
                              src/Twist.cpp
                              src/Utils.cpp
                              src/VectorDynSize.cpp
                              src/Wrench.cpp
                              src/PrivateUtils.cpp
                              src/SparseMatrix.cpp
                              src/Triplets.cpp
                              src/CubicSpline.cpp
                              src/SO3Utils.cpp)

SOURCE_GROUP("Source Files" FILES ${IDYNTREE_CORE_EXP_SOURCES})
SOURCE_GROUP("Header Files" FILES ${IDYNTREE_CORE_EXP_HEADERS})

# Check if this does not break existing build
# reason: avoid including with <iDynTree/Core/**> inside .cpp but using directly
# "**" which clearly states the difference between in-library files and external files
include_directories(include/iDynTree/Core)

set(libraryname idyntree-core)

add_library(${libraryname} ${IDYNTREE_CORE_EXP_SOURCES} ${IDYNTREE_CORE_EXP_HEADERS})
add_library(iDynTree::${libraryname} ALIAS ${libraryname})

if (DEFINED CMAKE_COMPILER_IS_GNUCXX)
  if(${CMAKE_COMPILER_IS_GNUCXX} AND ${CMAKE_CXX_COMPILER_VERSION} VERSION_LESS 5)
    target_compile_options(idyntree-core INTERFACE ${CXX11_FLAGS})
  endif()
endif()

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")
target_link_libraries(${libraryname} PRIVATE Eigen3::Eigen)

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
generate_export_header(${libraryname} EXPORT_FILE_NAME CoreExport.h)
list(APPEND IDYNTREE_CORE_EXP_HEADERS ${CMAKE_CURRENT_BINARY_DIR}/CoreExport.h)
target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>")

set_property(TARGET ${libraryname} PROPERTY PUBLIC_HEADER ${IDYNTREE_CORE_EXP_HEADERS})

target_compile_options(${libraryname} PRIVATE ${IDYNTREE_WARNING_FLAGS})

if(MSVC)
   add_definitions(-D_USE_MATH_DEFINES)
endif()

target_compile_features(${libraryname} PUBLIC cxx_attribute_deprecated)

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
