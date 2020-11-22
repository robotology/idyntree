set(libraryname idyntree-inverse-kinematics)

set(IDYN_TREE_IK_SOURCES src/ConvexHullHelpers.cpp
                         src/BoundingBoxHelpers.cpp
                         src/InverseKinematics.cpp)
set(IDYN_TREE_IK_HEADERS include/iDynTree/ConvexHullHelpers.h
                         include/iDynTree/BoundingBoxHelpers.h
                         include/iDynTree/InverseKinematics.h)

if(IDYNTREE_USES_IPOPT)
    set(PRIVATE_IDYN_TREE_IK_SOURCES src/InverseKinematicsNLP.cpp
                                     src/InverseKinematicsData.cpp
                                     src/TransformConstraint.cpp)
    set(PRIVATE_IDYN_TREE_IK_HEADERS include/private/InverseKinematicsNLP.h
                                     include/private/InverseKinematicsData.h
                                     include/private/TransformConstraint.h)
    source_group("Private\\Header Files" FILES ${PRIVATE_IDYN_TREE_IK_HEADERS})
    source_group("Private\\Source Files" FILES ${PRIVATE_IDYN_TREE_IK_SOURCES})

    add_library(${libraryname} ${IDYN_TREE_IK_HEADERS} ${IDYN_TREE_IK_SOURCES}
                               ${PRIVATE_IDYN_TREE_IK_SOURCES} ${PRIVATE_IDYN_TREE_IK_HEADERS})
else()
    add_library(${libraryname} ${IDYN_TREE_IK_HEADERS} ${IDYN_TREE_IK_SOURCES})
endif()

add_library(iDynTree::${libraryname} ALIAS ${libraryname})

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")
target_include_directories(${libraryname} PRIVATE include/private)

target_link_libraries(${libraryname} PUBLIC idyntree-core idyntree-high-level
                                     PRIVATE Eigen3::Eigen)

if(IDYNTREE_USES_IPOPT)
    target_compile_definitions(${libraryname} PRIVATE IDYNTREE_USES_IPOPT)
    target_compile_definitions(${libraryname} PRIVATE ${IPOPT_DEFINITIONS})
    target_include_directories(${libraryname} PRIVATE ${IPOPT_INCLUDE_DIRS})
    target_link_libraries(${libraryname} PRIVATE ${IPOPT_LIBRARIES})
endif()

set_property(TARGET ${libraryname} PROPERTY PUBLIC_HEADER ${IDYN_TREE_IK_HEADERS})

install(TARGETS ${libraryname}
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})


if(IDYNTREE_COMPILE_TESTS AND IDYNTREE_USES_IPOPT)
  add_subdirectory(tests)
endif()

