# Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

project(iDynTree_Sensors CXX)

# share headers with all iDynTree targets
set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include")
get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)

include_directories(${IDYNTREE_TREE_INCLUDE_DIRS})

set(IDYNTREE_SENSORS_HEADERS include/iDynTree/Sensors/AllSensorsTypes.h
                             include/iDynTree/Sensors/Sensors.h
                             include/iDynTree/Sensors/SixAxisForceTorqueSensor.h
                             include/iDynTree/Sensors/SixAxisFTSensor.h
                             include/iDynTree/Sensors/GyroscopeSensor.h
                             include/iDynTree/Sensors/AccelerometerSensor.h
                             include/iDynTree/Sensors/ThreeAxisAngularAccelerometerSensor.h
                             include/iDynTree/Sensors/ThreeAxisForceTorqueContactSensor.h
                             include/iDynTree/Sensors/PredictSensorsMeasurements.h
                             include/iDynTree/Sensors/ModelSensorsTransformers.h)

set(IDYNTREE_SENSORS_SOURCES src/Sensors.cpp
                             src/SixAxisForceTorqueSensor.cpp
                             src/AccelerometerSensor.cpp
                             src/GyroscopeSensor.cpp
                             src/ThreeAxisAngularAccelerometerSensor.cpp
                             src/ThreeAxisForceTorqueContactSensor.cpp
                             src/PredictSensorsMeasurements.cpp
                             src/ModelSensorsTransformers.cpp)


SOURCE_GROUP("Source Files" FILES ${IDYNTREE_SENSORS_SOURCES})
SOURCE_GROUP("Header Files" FILES ${IDYNTREE_SENSORS_HEADERS})

set(libraryname idyntree-sensors)

add_library(${libraryname} ${IDYNTREE_SENSORS_SOURCES} ${IDYNTREE_SENSORS_HEADERS} ${IDYNTREE_SENSORS_PRIVATE_INCLUDES})
add_library(iDynTree::${libraryname} ALIAS ${libraryname})

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>"
                                          PRIVATE ${EIGEN3_INCLUDE_DIR})

#include system headers
include_directories(SYSTEM AFTER ${orocos_kdl_INCLUDE_DIRS}
                                 ${EIGEN3_INCLUDE_DIR})

target_link_libraries(${libraryname} idyntree-core idyntree-model)

# Ensure that build include directories are always included before system ones
get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)
target_include_directories(${libraryname} BEFORE PRIVATE ${IDYNTREE_TREE_INCLUDE_DIRS})

target_compile_options(${libraryname} PRIVATE ${IDYNTREE_WARNING_FLAGS})

set_property(TARGET ${libraryname} PROPERTY PUBLIC_HEADER ${IDYNTREE_SENSORS_HEADERS})

install(TARGETS ${libraryname}
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/Sensors
        PRIVATE_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/Sensors/Impl)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})
set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)

if(IDYNTREE_COMPILE_TESTS)
    add_subdirectory(tests)
endif(IDYNTREE_COMPILE_TESTS)
