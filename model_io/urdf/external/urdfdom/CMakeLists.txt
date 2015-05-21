cmake_minimum_required( VERSION 2.8 FATAL_ERROR )
project (urdfdom CXX C)

set (URDF_MAJOR_VERSION 0)
set (URDF_MINOR_VERSION 3)
set (URDF_PATCH_VERSION 0)

set (URDF_VERSION ${URDF_MAJOR_VERSION}.${URDF_MINOR_VERSION}.${URDF_PATCH_VERSION})

message (STATUS "${PROJECT_NAME} version ${URDF_VERSION}")

include(GNUInstallDirs)

# hack: by default this would be 'lib/x86_64-linux-gnu'
set(CMAKE_INSTALL_LIBDIR lib)

# we are building this internal copy only on windows
# if we are not finding urdfdom, so to avoid to depend
# on Boost we use plain pointers
add_definitions(-DURDF_USE_PLAIN_POINTERS)

# This shouldn't be necessary, but there has been trouble
# with MSVC being set off, but MSVCXX ON.
if(MSVC OR MSVC90 OR MSVC10)
  set(MSVC ON)
endif (MSVC OR MSVC90 OR MSVC10)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# CMake options
option(URDFDOM_DO_NOT_USE_CONSOLEBRIDGE "If true do not use console_bridge for output messages, and rely on standard C functions." TRUE)
option(URDFDOM_DO_NOT_INSTALL_URDFPARSERPY "If true do not install the urdf_parser_py python package through CMake." TRUE)

if(URDFDOM_DO_NOT_USE_CONSOLEBRIDGE)
    add_definitions(-DURDFDOM_DO_NOT_USE_CONSOLEBRIDGE)
endif()

# Try to find a TinyXMLConfig.cmake file
find_package(TinyXML CONFIG QUIET)
# if a TinyXML cmake config file is not found, fallback
# to use the FindTinyXML.cmake file
if(NOT TinyXML_FOUND)
   find_package(TinyXML REQUIRED)
endif()
include_directories(SYSTEM ${TinyXML_INCLUDE_DIRS})

#find_package(urdfdom_headers)
#include_directories(SYSTEM ${urdfdom_headers_INCLUDE_DIRS})
include_directories(SYSTEM ../urdfdom_headers/urdf_model/include
                           ../urdfdom_headers/urdf_exception/include
                           ../urdfdom_headers/urdf_sensor/include
                           ../urdfdom_headers/urdf_world/include
                           ../urdfdom_headers/urdf_model_state/include)

if (NOT URDFDOM_DO_NOT_USE_CONSOLEBRIDGE)
    find_package(console_bridge REQUIRED)
    include_directories(SYSTEM ${console_bridge_INCLUDE_DIRS})
    link_directories(${console_bridge_LIBRARY_DIRS})
endif()

#In Visual Studio a special postfix for
#libraries compiled in debug is used
if(MSVC)
set(CMAKE_DEBUG_POSTFIX "d")
endif(MSVC)

add_subdirectory(urdf_parser)

set(PKG_NAME ${PROJECT_NAME})
set(PKG_LIBRARIES urdfdom_model urdfdom_world)
if(NOT URDFDOM_DO_NOT_USE_CONSOLEBRIDGE)
   set(PKG_DEPENDS urdfdom_headers console_bridge)
else()
   set(PKG_DEPENDS urdfdom_headers)
endif()
set(cmake_conf_file "cmake/urdfdom-config.cmake")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/${cmake_conf_file}.in" "${CMAKE_BINARY_DIR}/${cmake_conf_file}" @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/${cmake_conf_file}
  DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake/ COMPONENT cmake)

# If the proper option is selected by the user
# avoid trying to install urdf_parser_py via CMake
if (NOT URDFDOM_DO_NOT_INSTALL_URDFPARSERPY)
    add_subdirectory(urdf_parser_py)
endif()

message(STATUS "Configuration successful. Type make to compile urdfdom")
