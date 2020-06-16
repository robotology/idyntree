# Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

macro(IDYNTREE_ADD_TOOL tool_name tool_code)
    add_executable(${tool_name} ${tool_code})
    target_link_libraries(${tool_name} PRIVATE idyntree-core
                                               idyntree-model
                                               idyntree-high-level
                                               idyntree-modelio-urdf
                                               idyntree-visualization
                                               Eigen3::Eigen)
    target_compile_options(${tool_name} PRIVATE ${IDYNTREE_WARNING_FLAGS})
    install(TARGETS ${tool_name} DESTINATION bin)
endmacro(IDYNTREE_ADD_TOOL tool_name tool_code)

idyntree_add_tool(idyntree-model-info idyntree-model-info.cpp)

if(IDYNTREE_USES_IRRLICHT)
    idyntree_add_tool(idyntree-model-view idyntree-model-view.cpp)
endif()

if(IDYNTREE_USES_ICUB_MAIN)
  add_definitions(-D_USE_MATH_DEFINES)
  add_executable(urdf2dh urdf2dh)
  target_link_libraries(urdf2dh iKin idyntree-core idyntree-model idyntree-icub idyntree-high-level)
  install(TARGETS urdf2dh DESTINATION bin)
endif()

if (IDYNTREE_USES_YARP AND IDYNTREE_USES_QT5 AND IDYNTREE_USES_ICUB_MAIN)
    if (IDYNTREE_BUILD_IK)
        add_subdirectory(idyntree-sole-gui)
    endif()
    add_subdirectory(plotter)
endif()

if (IDYNTREE_USES_YARP)
    add_subdirectory(yarprobotstatepublisher)
endif ()
