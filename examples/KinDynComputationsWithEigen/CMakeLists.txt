# Copyright: 2017 Fondazione Istituto Italiano di Tecnologia
# Author: Silvio Traversaro
# CopyPolicy: Released under the terms of the GNU LGPL v2.0+.
#

cmake_minimum_required(VERSION 2.8.11)

project(KinDynComputationsWithEigen)

find_package(iDynTree REQUIRED)
# Note that while Eigen3 is using internally by iDynTree
# its public (installed) headers do not use it, so if you
# want to use Eigen3 in your project you need to find and include it
find_package(Eigen3 REQUIRED)

SET(folder_source main.cpp)

source_group("Source Files" FILES ${folder_source})

add_executable(${PROJECT_NAME} ${folder_source})

target_include_directories(${PROJECT_NAME} PUBLIC ${EIGEN3_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} ${iDynTree_LIBRARIES})


