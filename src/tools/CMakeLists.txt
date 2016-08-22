macro(IDYNTREE_ADD_TOOL_YARP_KDL tool_name tool_code)
    add_executable(${tool_name} ${tool_code})
    target_link_libraries(${tool_name} idyntree-core
                                       idyntree-regressors
                                       idyntree-modelio-urdf-kdl
                                       idyntree-modelio-symoro)
    include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR} ${orocos_kdl_INCLUDE_DIRS})
    install(TARGETS ${tool_name} DESTINATION bin)
endmacro(IDYNTREE_ADD_TOOL_YARP_KDL tool_name tool_code)

macro(IDYNTREE_ADD_TOOL tool_name tool_code)
    add_executable(${tool_name} ${tool_code})
    target_link_libraries(${tool_name} idyntree-core
                                       idyntree-model
                                       idyntree-high-level
                                       idyntree-modelio-urdf
                                       idyntree-visualization)
    include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
    install(TARGETS ${tool_name} DESTINATION bin)
endmacro(IDYNTREE_ADD_TOOL tool_name tool_code)

idyntree_add_tool(idyntree-model-info idyntree-model-info.cpp)

if(IDYNTREE_USES_IRRLICHT)
    idyntree_add_tool(idyntree-model-view idyntree-model-view.cpp)
endif()

if(IDYNTREE_USES_ICUB_MAIN AND IDYNTREE_USES_KDL)
    idyntree_add_tool_yarp_kdl(par2urdf par2urdf.cpp)
endif()

if(IDYNTREE_USES_ICUB_MAIN AND IDYNTREE_USES_KDL)
  find_package(YARP)
  find_package(ICUB)
  add_definitions(-D_USE_MATH_DEFINES)
  add_executable(urdf2dh urdf2dh)
  target_include_directories(urdf2dh PUBLIC ${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
  target_link_libraries(urdf2dh iKin idyntree-core idyntree-regressors  idyntree-modelio-urdf-kdl idyntree-modelio-ikin)
  install(TARGETS urdf2dh DESTINATION bin)
endif()
