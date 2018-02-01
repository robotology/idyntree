# Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

macro(IDYNTREE_ADD_TOOL_YARP_KDL tool_name tool_code)
    add_executable(${tool_name} ${tool_code})
    target_link_libraries(${tool_name} idyntree-core
                                       idyntree-regressors
                                       idyntree-modelio-urdf-kdl
                                       idyntree-modelio-symoro)
if(NOT CMAKE_VERSION VERSION_LESS 2.8.12)
    target_compile_options(${tool_name} PRIVATE ${IDYNTREE_WARNING_FLAGS})
endif()
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
if(NOT CMAKE_VERSION VERSION_LESS 2.8.12)
    target_compile_options(${tool_name} PRIVATE ${IDYNTREE_WARNING_FLAGS})
endif()
    include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
    install(TARGETS ${tool_name} DESTINATION bin)
endmacro(IDYNTREE_ADD_TOOL tool_name tool_code)

idyntree_add_tool(idyntree-model-info idyntree-model-info.cpp)

if(IDYNTREE_USES_IRRLICHT)
    idyntree_add_tool(idyntree-model-view idyntree-model-view.cpp)
endif()

if(IDYNTREE_USES_ICUB_MAIN AND IDYNTREE_USES_KDL AND IDYNTREE_USES_SYMORO_PAR)
    idyntree_add_tool_yarp_kdl(par2urdf par2urdf.cpp)
endif()

if(IDYNTREE_USES_ICUB_MAIN)
  find_package(YARP)
  find_package(ICUB)
  add_definitions(-D_USE_MATH_DEFINES)
  add_executable(urdf2dh urdf2dh)
  target_include_directories(urdf2dh PUBLIC ${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
  target_link_libraries(urdf2dh iKin idyntree-core idyntree-regressors  idyntree-model idyntree-icub idyntree-high-level)
  install(TARGETS urdf2dh DESTINATION bin)
endif()

if (IDYNTREE_USES_YARP AND IDYNTREE_USES_QT5 AND IDYNTREE_USES_ICUB_MAIN)
    if (IDYNTREE_BUILD_IK)
        add_subdirectory(idyntree-sole-gui)
    endif()
    add_subdirectory(plotter)
endif()
