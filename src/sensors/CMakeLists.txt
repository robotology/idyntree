# Copyright: (C) 2015 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(iDynTree_Sensors CXX)

# share headers with all iDynTree targes
set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include")
get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)

include_directories(${IDYNTREE_TREE_INCLUDE_DIRS})

#include system headers
include_directories(SYSTEM ${orocos_kdl_INCLUDE_DIRS}
                            ${EIGEN3_INCLUDE_DIR})

set(IDYNTREE_SENSORS_HEADERS include/iDynTree/Sensors/Sensors.h
                             include/iDynTree/Sensors/SixAxisFTSensor.h
                             include/iDynTree/Sensors/Gyroscope.h
                             include/iDynTree/Sensors/Accelerometer.h
                             include/iDynTree/Sensors/PredictSensorsMeasurements.h)

#file (GLOB IDYNTREE_BASE_PRIVATE_INCLUDES)

set(IDYNTREE_SENSORS_SOURCES src/Sensors.cpp
                             src/SixAxisFTSensor.cpp
                             src/Accelerometer.cpp
                             src/Gyroscope.cpp
                             src/PredictSensorsMeasurements.cpp)

               
SOURCE_GROUP("Source Files" FILES ${IDYNTREE_SENSORS_SOURCES})
SOURCE_GROUP("Header Files" FILES ${IDYNTREE_SENSORS_HEADERS})
#SOURCE_GROUP("Header Files\\Private" FILES ${IDYNTREE_SENSORS_PRIVATE_INCLUDES})

set(libraryname idyntree-sensors)

add_library(${libraryname} ${IDYNTREE_SENSORS_SOURCES} ${IDYNTREE_SENSORS_HEADERS} ${IDYNTREE_SENSORS_PRIVATE_INCLUDES})

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
                                                "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>"
                                         PRIVATE ${EIGEN3_INCLUDE_DIR})

target_link_libraries(${libraryname} idyntree-core)

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
