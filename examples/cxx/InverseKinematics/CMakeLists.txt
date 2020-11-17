# Copyright: 2018 Fondazione Istituto Italiano di Tecnologia
# Author: Silvio Traversaro
# CopyPolicy: Released under the terms of the MIT License
#

cmake_minimum_required(VERSION 3.5)

project(iDynTreeExampleInverseKinematics)

find_package(iDynTree REQUIRED)

source_group("Source Files" FILES iDynTreeExampleInverseKinematics.cpp)

add_executable(${PROJECT_NAME} iDynTreeExampleInverseKinematics.cpp)
target_link_libraries(${PROJECT_NAME} ${iDynTree_LIBRARIES})

# Use M_PI also on Windows
add_definitions(-D_USE_MATH_DEFINES)

# To avoid vendoring the URDF files used for the example, let's download it on the fly
if(NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/kr16_2-buffer.urdf)
  file(DOWNLOAD https://raw.githubusercontent.com/ros-industrial/kuka_experimental/indigo-devel/kuka_kr16_support/urdf/kr16_2.urdf ${CMAKE_CURRENT_BINARY_DIR}/kr16_2-buffer.urdf)
endif()

# For multiple config generators, the binary is not in the CMAKE_CURRENT_BINARY_DIR
# let's make sure that we copy the file in the executable folder, using the TARGET_FILE_DIR
# expressor generator ( https://cmake.org/cmake/help/latest/manual/cmake-generator-expressions.7.html )
file(GENERATE OUTPUT $<TARGET_FILE_DIR:${PROJECT_NAME}>/kr16_2.urdf
     INPUT ${CMAKE_CURRENT_BINARY_DIR}/kr16_2-buffer.urdf)


