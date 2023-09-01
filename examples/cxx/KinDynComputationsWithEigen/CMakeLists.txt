# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.5)

project(KinDynComputationsWithEigen)

find_package(iDynTree REQUIRED)

# Note that while Eigen3 is using internally by iDynTree
# its public (installed) headers do not use it, so if you
# want to use Eigen3 in your project you need to find and include it
find_package(Eigen3 CONFIG REQUIRED)

SET(folder_source main.cpp)

source_group("Source Files" FILES ${folder_source})

add_executable(${PROJECT_NAME} ${folder_source})

message(STATUS "iDynTree_LIBRARIES : ${iDynTree_LIBRARIES}")
target_link_libraries(${PROJECT_NAME} PRIVATE ${iDynTree_LIBRARIES} Eigen3::Eigen)


