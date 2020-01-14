# Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Silvio Traversaro <silvio.traversaro@iit.it>

project(yarprobotstatepublisher)

add_executable(${PROJECT_NAME}
    include/robotstatepublisher.h
    src/main.cpp
    src/robotstatepublisher.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/include)

target_link_libraries(${PROJECT_NAME}
    YARP::YARP_init
    YARP::YARP_os
    YARP::YARP_dev
    YARP::YARP_rosmsg
    idyntree-high-level
    idyntree-yarp)

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
