# Copyright: (C) 2015 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(iDynTree_Sensors CXX)

# share headers with all iDynTree targets
set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include")
get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)

include_directories(${IDYNTREE_TREE_INCLUDE_DIRS})

set(IDYNTREE_SENSORS_HEADERS include/iDynTree/Sensors/Sensors.h
                             include/iDynTree/Sensors/SixAxisFTSensor.h
                             include/iDynTree/Sensors/GyroscopeSensor.h
                             include/iDynTree/Sensors/AccelerometerSensor.h
                             include/iDynTree/Sensors/PredictSensorsMeasurements.h
                             include/iDynTree/Sensors/ModelSensorsTransformers.h)

set(IDYNTREE_SENSORS_SOURCES src/Sensors.cpp
                             src/SixAxisFTSensor.cpp
                             src/AccelerometerSensor.cpp
                             src/GyroscopeSensor.cpp
                             src/PredictSensorsMeasurements.cpp
                             src/ModelSensorsTransformers.cpp)


SOURCE_GROUP("Source Files" FILES ${IDYNTREE_SENSORS_SOURCES})
SOURCE_GROUP("Header Files" FILES ${IDYNTREE_SENSORS_HEADERS})

set(libraryname idyntree-sensors)

add_library(${libraryname} ${IDYNTREE_SENSORS_SOURCES} ${IDYNTREE_SENSORS_HEADERS} ${IDYNTREE_SENSORS_PRIVATE_INCLUDES})

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>"
                                          PRIVATE ${EIGEN3_INCLUDE_DIR})

#include system headers
include_directories(SYSTEM AFTER ${orocos_kdl_INCLUDE_DIRS}
                                 ${EIGEN3_INCLUDE_DIR})

target_link_libraries(${libraryname} idyntree-core idyntree-model)

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
