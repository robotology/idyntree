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

# set the default build type
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# If compiler support symbol visibility, enable it.
include(CheckCCompilerFlag)
check_c_compiler_flag(-fvisibility=hidden HAS_VISIBILITY)
if (HAS_VISIBILITY)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden")
endif()

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

if(NOT CMAKE_VERSION VERSION_LESS 2.8.6)
  # If CMake is version 2.8.6 or later, properly produce a version configuration files
  set(cmake_conf_version_file "cmake/urdfdom-config-version.cmake")
  # Use write_basic_package_version_file to generate a ConfigVersion file that
  # allow users of urdfdom to specify the API or version to depend on
  # TODO: keep this instruction until deprecate Ubuntu/Precise and update with
  # https://github.com/Kitware/CMake/blob/v2.8.8/Modules/CMakePackageConfigHelpers.cmake
  include(WriteBasicConfigVersionFile)
  write_basic_config_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${cmake_conf_version_file}
    VERSION "${URDF_VERSION}"
    COMPATIBILITY SameMajorVersion)

  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${cmake_conf_version_file}
          DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake/
          COMPONENT cmake)

endif()

# Make the package config file
if (NOT MSVC)
  set(PKG_DESC "Unified Robot Description Format")
  set(PKG_DEPENDS "urdfdom_headers console_bridge") # make the list separated by spaces instead of ;
  set(PKG_URDF_LIBS "-lurdfdom_model -lurdfdom_world")
  set(pkg_conf_file "cmake/pkgconfig/urdfdom.pc")
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/${pkg_conf_file}.in" "${CMAKE_BINARY_DIR}/${pkg_conf_file}" @ONLY)
  install(FILES ${CMAKE_BINARY_DIR}/${pkg_conf_file}
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig/ COMPONENT pkgconfig)
endif()

# If the proper option is selected by the user
# avoid trying to install urdf_parser_py via CMake
if (NOT URDFDOM_DO_NOT_INSTALL_URDFPARSERPY)
    add_subdirectory(urdf_parser_py)
endif()

message(STATUS "Configuration successful. Type make to compile urdfdom")

SET_DIRECTORY_PROPERTIES(PROPERTIES
  ADDITIONAL_MAKE_CLEAN_FILES ${CMAKE_BINARY_DIR}/cmake/urdfdom-config.cmake
  ADDITIONAL_MAKE_CLEAN_FILES ${CMAKE_BINARY_DIR}/cmake/pkgconfig/urdfdom.pc)
