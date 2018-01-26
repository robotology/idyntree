cmake_minimum_required(VERSION 2.8.12)

project(iDynTree_InverseKinematics CXX)

set(libraryname idyntree-inverse-kinematics)

set(IDYN_TREE_IK_SOURCES src/ConvexHullHelpers.cpp
                         src/BoundingBoxHelpers.cpp
                         src/InverseKinematics.cpp)
set(IDYN_TREE_IK_HEADERS include/iDynTree/ConvexHullHelpers.h
                         include/iDynTree/BoundingBoxHelpers.h
                         include/iDynTree/InverseKinematics.h)

set(PRIVATE_IDYN_TREE_IK_SOURCES src/InverseKinematicsNLP.cpp
                                 src/InverseKinematicsData.cpp
                                 src/TransformConstraint.cpp)
set(PRIVATE_IDYN_TREE_IK_HEADERS include/private/InverseKinematicsNLP.h
                                 include/private/InverseKinematicsData.h
                                 include/private/TransformConstraint.h)

source_group("Private\\Header Files" FILES ${PRIVATE_IDYN_TREE_IK_HEADERS})
source_group("Private\\Source Files" FILES ${PRIVATE_IDYN_TREE_IK_SOURCES})

idyntree_enable_cxx11()

add_library(${libraryname} ${IDYN_TREE_IK_HEADERS} ${IDYN_TREE_IK_SOURCES}
                           ${PRIVATE_IDYN_TREE_IK_SOURCES} ${PRIVATE_IDYN_TREE_IK_HEADERS})

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include;${IDYNTREE_TREE_INCLUDE_DIRS}>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>")
target_include_directories(${libraryname} PRIVATE include/private)


# Probably SYSTEM is not compatible with cmake 2.8.12, but I think as YARP requires
# CMake 3.0 we can increase the min version
target_include_directories(${libraryname} SYSTEM PUBLIC ${IPOPT_INCLUDE_DIRS})
target_include_directories(${libraryname} SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(${libraryname} idyntree-core idyntree-high-level ${IPOPT_LIBRARIES})

target_compile_definitions(${libraryname} PRIVATE ${IPOPT_DEFINITIONS})

set_property(TARGET ${libraryname} PROPERTY PUBLIC_HEADER ${IDYN_TREE_IK_HEADERS})

install(TARGETS ${libraryname}
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})
set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)


if(IDYNTREE_COMPILE_TESTS)
  add_subdirectory(tests)
endif()

