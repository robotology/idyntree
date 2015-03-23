# Copyright: 2015 Fondazione Istituto Italiano di Tecnologia
# Author: Silvio Traversaro
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

CMAKE_MINIMUM_REQUIRED(VERSION 2.8.11)

SET(PROJECTNAME walkmanLimbJTSRegressor)

PROJECT(${PROJECTNAME})

FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(iDynTree REQUIRED)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})

SET(folder_source main.cpp)

SOURCE_GROUP("Source Files" FILES ${folder_source})

message(STATUS "iDynTree_INCLUDE_DIRS is ${iDynTree_INCLUDE_DIRS}")

INCLUDE_DIRECTORIES(${iDynTree_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})


ADD_EXECUTABLE(${PROJECTNAME} ${folder_source})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES}
                                     ${iDynTree_LIBRARIES})


