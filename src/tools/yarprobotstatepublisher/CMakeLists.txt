# Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Silvio Traversaro <silvio.traversaro@iit.it>

add_executable(yarprobotstatepublisher
    include/robotstatepublisher.h
    src/main.cpp
    src/robotstatepublisher.cpp)

target_include_directories(yarprobotstatepublisher PRIVATE include)

target_link_libraries(yarprobotstatepublisher
    YARP::YARP_init
    YARP::YARP_os
    YARP::YARP_dev
    YARP::YARP_rosmsg
    idyntree-high-level
    idyntree-yarp)

install(TARGETS yarprobotstatepublisher DESTINATION bin)
