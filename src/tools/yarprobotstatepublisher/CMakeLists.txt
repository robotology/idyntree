# Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia (IIT)  
# All Rights Reserved.
# Authors: Silvio Traversaro <silvio.traversaro@iit.it>

idyntree_enable_cxx11()

project(yarprobotstatepublisher)

set(ROS_IDL_FILES JointState.msg)
yarp_add_idl(IDL_GEN_FILES ${ROS_IDL_FILES})
message("IDL_GEN_FILES : " ${IDL_GEN_FILES})
add_executable(${PROJECT_NAME} include/robotstatepublisher.h src/main.cpp src/robotstatepublisher.cpp ${IDL_GEN_FILES})
target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/include
                                                     ${YARP_INCLUDE_DIRS})
# Workaround for yarp_add_idl problems
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} idyntree-high-level idyntree-yarp)
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
