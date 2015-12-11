include_directories(include)

get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)
include_directories(SYSTEM ${IDYNTREE_TREE_INCLUDE_DIRS})

# share headers with all iDynTree targes
set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/urdf_parser/include")
set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/../urdfdom_headers/urdf_model/include")
set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/../urdfdom_headers/urdf_exception/include")


#add_library(iDynTree_priv_urdfdom_world SHARED src/pose.cpp src/model.cpp src/link.cpp src/joint.cpp src/world.cpp)
#target_link_libraries(iDynTree_priv_urdfdom_world ${TinyXML_LIBRARIES})
#set_target_properties(iDynTree_priv_urdfdom_world PROPERTIES SOVERSION 0.3)

add_library(iDynTree_priv_urdfdom_model STATIC src/pose.cpp
                                               src/model.cpp
                                               src/link.cpp
                                               src/joint.cpp
											   # comment out unused files
                                               #src/urdf_sensor.cpp
                                               #src/urdf_model_state.cpp
                                               #src/twist.cpp
											   )
target_link_libraries(iDynTree_priv_urdfdom_model ${TinyXML_LIBRARIES})
set_target_properties(iDynTree_priv_urdfdom_model PROPERTIES SOVERSION 0.3)

target_include_directories(iDynTree_priv_urdfdom_model PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                              "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

set(URDFDOM_MODEL_PUBLIC_HEADERS include/urdf_parser/urdf_parser.h
                                 include/urdf_parser/outputdecl.h
                                 include/urdf_parser/exportdecl.h )

set_property(TARGET ${libraryname} PROPERTY PUBLIC_HEADER ${URDFDOM_MODEL_PUBLIC_HEADERS})


# --------------------------------

#add_executable(check_urdf src/check_urdf.cpp)
#target_link_libraries(check_urdf iDynTree_priv_urdfdom_model)

#add_executable(urdf_to_graphiz src/urdf_to_graphiz.cpp)
#target_link_libraries(urdf_to_graphiz iDynTree_priv_urdfdom_model)

#add_executable(urdf_mem_test test/memtest.cpp)
#target_link_libraries(urdf_mem_test iDynTree_priv_urdfdom_model)


INSTALL(TARGETS iDynTree_priv_urdfdom_model DESTINATION ${CMAKE_INSTALL_LIBDIR})
#INSTALL(TARGETS iDynTree_priv_urdfdom_world DESTINATION ${CMAKE_INSTALL_LIBDIR})
#INSTALL(TARGETS check_urdf urdf_to_graphiz urdf_mem_test
#  DESTINATION ${CMAKE_INSTALL_BINDIR})
INSTALL(DIRECTORY include/urdf_parser DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
#INSTALL(DIRECTORY ../urdfdom_headers/urdf_model/include/urdf_model DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
#INSTALL(DIRECTORY ../urdfdom_headers/urdf_exception/include/urdf_exception  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
#INSTALL(DIRECTORY ../urdfdom_headers/urdf_sensor/include/urdf_sensor DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
#INSTALL(DIRECTORY ../urdfdom_headers/urdf_world/include/urdf_world  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
#INSTALL(DIRECTORY ../urdfdom_headers/urdf_model_state/include/urdf_model_state  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(TARGETS iDynTree_priv_urdfdom_model
        EXPORT iDynTree
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}/urdf_parser" COMPONENT dev)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS iDynTree_priv_urdfdom_model)

