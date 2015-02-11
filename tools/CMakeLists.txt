macro(IDYNTREE_ADD_TOOL tool_name tool_code)
    add_executable(${tool_name} ${tool_code})
    target_link_libraries(${tool_name} iKin idyntree-core idyntree-regressors  idyntree-model-io)
    install(TARGETS ${tool_name} DESTINATION bin)
endmacro(IDYNTREE_ADD_TOOL tool_name tool_code)

get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)

include_directories(${IDYNTREE_TREE_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${orocos_kdl_INCLUDE_DIRS})

idyntree_add_tool(par2urdf par2urdf.cpp)

if(IDYNTREE_USE_ICUB_MAIN)
  find_package(YARP)
  find_package(ICUB)
  add_executable(urdf2dh urdf2dh)
  target_include_directories(urdf2dh PUBLIC ${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
  target_link_libraries(urdf2dh iKin idyntree-core idyntree-regressors  idyntree-model-io)
  install(TARGETS urdf2dh DESTINATION bin)
endif()