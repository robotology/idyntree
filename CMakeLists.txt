cmake_minimum_required( VERSION 2.8 FATAL_ERROR )
project (kdl_format_io CXX)

set (KDL_FORMAT_IO_VERSION_MAJOR 0)
set (KDL_FORMAT_IO_VERSION_MINOR 0)
set (KDL_FORMAT_IO_VERSION_PATCH 1)

set (KDL_FORMAT_IO_VERSION ${KDL_FORMAT_IO_VERSION_MAJOR}.${KDL_FORMAT_IO_VERSION_MINOR}.${KDL_FORMAT_IO_VERSION_PATCH})

message (STATUS "${PROJECT_NAME} version ${KDL_FORMAT_IO_VERSION}")

# set the default build type
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# This shouldn't be necessary, but there has been trouble
# with MSVC being set off, but MSVCXX ON.
if(MSVC OR MSVC90 OR MSVC10)
  set(MSVC ON)
endif (MSVC OR MSVC90 OR MSVC10)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

#define the support for specific file formats
#by default they are all on, they are disabled if the necessary dependencies were not found

## URDF file format support
option(ENABLE_URDF "Enable support for URDF input/output" TRUE)

## SYMORO par file format support
option(ENABLE_SYMORO_PAR "Enable support for SYMORO par input" TRUE)

## KDL::CoDyCo::TreeSerialization support
option(ENABLE_SERIALIZATION_IO "Enable support for parsing and writing serialization (need kdl_codyco)" TRUE)

option(KDL_FORMAT_IO_DISABLE_RPATH "Disable RPATH for the library" FALSE)

find_package(orocos_kdl 1.2.3 QUIET)
if (NOT orocos_kdl_FOUND)
    SET(OROCOS_KDL_OLDVERSION On)
    find_package(orocos_kdl)
endif(NOT orocos_kdl_FOUND)

#support also for the old version of kdl cmake package
if(NOT orocos_kdl_FOUND)
   find_package(Orocos-KDL)
   if(NOT Orocos-KDL_FOUND)
      message(WARNING "KDL not found: neither orocos_kdl or Orocos-KDL cmake packages are available")
   else(NOT Orocos-KDL_FOUND)
      set(orocos_kdl_INCLUDE_DIRS ${Orocos-KDL_INCLUDE_DIRS})
      set(orocos_kdl_LIBRARY_DIRS ${Orocos-KDL_LIBRARY_DIRS})
      set(orocos_kdl_LIBRARIES ${Orocos-KDL_LIBRARIES})
      set(orocos_kdl_FOUND true)
      set(orocos_kdl_VERSION ${Orocos-KDL_VERSION})
      set(orocos_kdl_VERSION_MAJOR ${Orocos-KDL_VERSION_MAJOR})
      set(orocos_kdl_VERSION_MINOR ${Orocos-KDL_VERSION_MINOR})
      set(orocos_kdl_VERSION_PATCH ${Orocos-KDL_VERSION_PATCH})
   endif(NOT Orocos-KDL_FOUND)
endif(NOT orocos_kdl_FOUND)

add_definitions(-DOROCOS_KDL_VERSION_MAJOR=${orocos_kdl_VERSION_MAJOR})
add_definitions(-DOROCOS_KDL_VERSION_MINOR=${orocos_kdl_VERSION_MINOR})
add_definitions(-DOROCOS_KDL_VERSION_PATCH=${orocos_kdl_VERSION_PATCH})

include_directories(SYSTEM ${orocos_kdl_INCLUDE_DIRS})
set(KDL_FORMAT_IO_INCLUDE_DIRS "${KDL_FORMAT_IO_INCLUDE_DIRS}" ${orocos_kdl_INCLUDE_DIRS})

IF( ENABLE_URDF )
    find_package(TinyXML)
    IF( NOT TinyXML_FOUND )
        message("Disabling URDF support as no TinyXML was found")
        set(ENABLE_URDF FALSE)
    ENDIF()
ENDIF()

IF( ENABLE_URDF )
    find_package(urdfdom_headers)
    IF( NOT urdfdom_headers_CONFIG_INCLUDED )
        message("Disabling URDF support as no urdfdom_headers was found")
        set(ENABLE_URDF FALSE)
    ENDIF()
ENDIF()

IF( ENABLE_URDF )
    find_package(urdfdom)
    IF( NOT urdfdom_CONFIG_INCLUDED )
        message("Disabling URDF support as no urdfdom was found")
        set(ENABLE_URDF FALSE)
    ENDIF()
ENDIF()

IF(ENABLE_SERIALIZATION_IO)
    find_package(kdl_codyco)
    IF( NOT kdl_codyco_FOUND )
        message("Disabling Serialization as no kdl_codyco was found")
        set(ENABLE_SERIALIZATION_IO FALSE)
    ELSE()
         include_directories(SYSTEM ${kdl_codyco_INCLUDE_DIRS})
         set(KDL_FORMAT_IO_INCLUDE_DIRS ${KDL_FORMAT_IO_INCLUDE_DIRS} ${kdl_codyco_INCLUDE_DIRS})
         link_directories(${kdl_codyco_LIBRARY_DIRS})
         set(KDL_FORMAT_IO_LIBRARY_DIRS ${KDL_FORMAT_IO_LIBRARY_DIRS} ${kdl_codyco_LIBRARY_DIRS})
    ENDIF()
ENDIF()

include_directories(include)


if(ENABLE_URDF)
    include_directories(SYSTEM ${TinyXML_INCLUDE_DIRS})
    include_directories(SYSTEM ${urdfdom_INCLUDE_DIRS})
    include_directories(SYSTEM ${urdfdom_headers_INCLUDE_DIRS})
    include_directories(SYSTEM ${console_bridge_INCLUDE_DIRS})
    set(KDL_FORMAT_IO_INCLUDE_DIRS "${KDL_FORMAT_IO_INCLUDE_DIRS}" ${TinyXML_INCLUDE_DIRS} ${urdfdom_INCLUDE_DIRS} ${urdfdom_headers_INCLUDE_DIRS} ${console_bridge_INCLUDE_DIRS})

    link_directories(${console_bridge_LIBRARY_DIRS})
    set(KDL_FORMAT_IO_LIBRARY_DIRS "${KDL_FORMAT_IO_LIBRARY_DIRS}" ${console_bridge_LIBRARY_DIRS} ${urdfdom_LIBRARY_DIRS})

    set(URDF_SRCS src/converters/urdf_export.cpp
                  src/converters/urdf_import.cpp
                  src/converters/urdf_sensor_import.cpp)

    set(URDF_HPPS include/kdl_format_io/urdf_import.hpp
                  include/kdl_format_io/urdf_export.hpp
                  include/kdl_format_io/urdf_sensor_import.hpp)
    set(URDF_LIBS ${urdfdom_LIBRARIES} ${console_bridge_LIBRARIES} ${Boost_LIBRARIES})
endIF()

if(ENABLE_SYMORO_PAR)
    set(EXPR_PARSER_SRCS src/expression_parser/error.cpp
                     src/expression_parser/functions.cpp
                     src/expression_parser/parser.cpp
                     src/expression_parser/variablelist.cpp)
    set(SYMORO_PAR_SRCS src/converters/symoro_par_import.cpp ${EXPR_PARSER_SRCS})
    set(SYMORO_PAR_HPPS include/kdl_format_io/symoro_par_import.hpp include/kdl_format_io/symoro_par_model.hpp)
    if(ENABLE_SERIALIZATION_IO)
        set(SYMORO_PAR_HPPS ${SYMORO_PAR_HPPS} include/kdl_format_io/symoro_par_import_serialization.hpp)
        set(SYMORO_PAR_SRCS ${SYMORO_PAR_SRCS} src/converters/symoro_par_import_serialization.cpp)
    endif()
endif()

set(KDL_FORMAT_IO_HPPS ${SYMORO_PAR_HPPS} ${URDF_HPPS})

if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
    set(LIB_TYPE STATIC)
else()
    set(LIB_TYPE SHARED)
endif()

#Configure RPATH
set(CMAKE_MACOSX_RPATH 1) #enable RPATH on OSX. This also suppress warnings on CMake >= 3.0
# when building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
# set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
#This is relative RPATH for libraries built in the same project
#I assume that the directory is
# - install_dir/something for binaries
# - install_dir/lib for libraries
file(RELATIVE_PATH _rel_path "${CMAKE_INSTALL_PREFIX}/bin" "${CMAKE_INSTALL_PREFIX}/lib")
if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
 set(CMAKE_INSTALL_RPATH "@loader_path/${_rel_path}")
else()
 set(CMAKE_INSTALL_RPATH "\$ORIGIN/${_rel_path}")
endif()
# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE) #very important!


if(KDL_FORMAT_IO_DISABLE_RPATH)
    set(CMAKE_SKIP_BUILD_RPATH TRUE)
else(KDL_FORMAT_IO_DISABLE_RPATH)
    set(CMAKE_SKIP_BUILD_RPATH FALSE)
endif(KDL_FORMAT_IO_DISABLE_RPATH)

option(ENABLE_IKIN "Enable support for exporting iKin chains" FALSE)

if(ENABLE_IKIN)
  set(IKIN_SRCS src/converters/iKin_export.cpp)
endif()

add_library(kdl-format-io ${LIB_TYPE} ${URDF_SRCS} ${SYMORO_PAR_SRCS} ${KDL_FORMAT_IO_HPPS} ${IKIN_SRCS})

IF(ENABLE_SERIALIZATION_IO)
    target_link_libraries(kdl-format-io ${kdl_codyco_LIBRARIES} ${TinyXML_LIBRARIES} ${URDF_LIBS}  ${orocos_kdl_LIBRARIES})
    set(KDL_FORMAT_IO_LIBRARIES ${kdl_codyco_LIBRARIES} ${TinyXML_LIBRARIES} ${URDF_LIBS} ${orocos_kdl_LIBRARIES})
ELSE()
    target_link_libraries(kdl-format-io ${URDF_LIBS} ${TinyXML_LIBRARIES} ${orocos_kdl_LIBRARIES})
    set(KDL_FORMAT_IO_LIBRARIES ${URDF_LIBS} ${TinyXML_LIBRARIES} ${orocos_kdl_LIBRARIES})
ENDIF()

if(ENABLE_IKIN)
    find_package(YARP REQUIRED)
    find_package(ICUB REQUIRED)
    target_link_libraries(kdl-format-io ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${URDF_LIBS} ${TinyXML_LIBRARIES} ${orocos_kdl_LIBRARIES})
    set(KDL_FORMAT_IO_LIBRARIES  ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${URDF_LIBS} ${TinyXML_LIBRARIES} ${orocos_kdl_LIBRARIES})
endif()

IF( ENABLE_URDF AND ENABLE_SYMORO_PAR )
    add_executable(par2urdf src/utils/par2urdf.cpp)
    target_link_libraries(par2urdf kdl-format-io ${URDF_LIBS} ${orocos_kdl_LIBRARIES})
ENDIF()

IF( ENABLE_URDF AND ENABLE_IKIN )
    add_executable(urdf2dh src/utils/urdf2dh.cpp)
    target_link_libraries(urdf2dh kdl-format-io ${URDF_LIBS} ${orocos_kdl_LIBRARIES} ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${kdl_codyco_LIBRARIES})
ENDIF()

#add_executable(check_kdl_export src/check_kdl_export.cpp src/kdl_import.cpp src/kdl_export.cpp)
#target_link_libraries(check_kdl_export ${kdl_format_io_LIBRARIES} ${urdfdom_headers_LIBRARIES}  ${urdfdom_LIBRARIES} ${console_bridge_LIBRARIES} ${Boost_LIBRARIES} ${Orocos-KDL_LIBRARIES})

set_target_properties( kdl-format-io PROPERTIES
  SOVERSION "${KDL_FORMAT_IO_VERSION_MAJOR}.${KDL_FORMAT_IO_VERSION_MINOR}"
  VERSION "${KDL_FORMAT_IO_VERSION}"
  COMPILE_FLAGS "${CMAKE_CXX_FLAGS_ADD}"
  PUBLIC_HEADER "${KDL_FORMAT_IO_HPPS}"
  )

if(${CMAKE_MINIMUM_REQUIRED_VERSION} VERSION_GREATER "2.8.12")
  message(AUTHOR_WARNING "CMAKE_MINIMUM_REQUIRED_VERSION is now ${CMAKE_MINIMUM_REQUIRED_VERSION}. This check can be removed.")
endif()
if(CMAKE_VERSION VERSION_LESS 2.8.12)
  if(NOT KDL_FORMAT_IO_DISABLE_RPATH)
      set_target_properties(kdl-format-io PROPERTIES
          INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
      )
  endif(NOT KDL_FORMAT_IO_DISABLE_RPATH)
endif(CMAKE_VERSION VERSION_LESS 2.8.12)


INSTALL(TARGETS kdl-format-io
  EXPORT kdl_format_io-target
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  PUBLIC_HEADER DESTINATION include/kdl_format_io
)

#install utilities
if( ENABLE_URDF AND ENABLE_SYMORO_PAR )
    install(TARGETS par2urdf DESTINATION bin)
endif()

if( ENABLE_URDF AND ENABLE_IKIN )
    install(TARGETS urdf2dh DESTINATION bin)
endif()

EXPORT(TARGETS kdl-format-io FILE "${PROJECT_BINARY_DIR}/kdl_format_io-target.cmake")

EXPORT(PACKAGE kdl_format_io)

set(cmake_conf_file_raw "${CMAKE_CURRENT_SOURCE_DIR}/cmake/kdl_format_io-config.cmake.in")
set(cmake_conf_file "${PROJECT_BINARY_DIR}/kdl_format_io-config.cmake")
configure_file("${cmake_conf_file_raw}" "${cmake_conf_file}" @ONLY)
install(FILES ${cmake_conf_file} DESTINATION share/${PROJECT_NAME})

set(cmake_version_conf_file_raw "${CMAKE_CURRENT_SOURCE_DIR}/cmake/kdl_format_io-config-version.cmake.in")
set(cmake_version_conf_file "${PROJECT_BINARY_DIR}/kdl_format_io-config-version.cmake")
configure_file("${cmake_version_conf_file_raw}" "${cmake_version_conf_file}" @ONLY)
install(FILES ${cmake_version_conf_file} DESTINATION share/${PROJECT_NAME})

INSTALL(EXPORT kdl_format_io-target  DESTINATION share/kdl_format_io)

option(CREATE_TESTS "Create regression tests using CMake" FALSE)

IF(CREATE_TESTS)
    enable_testing()
    ADD_SUBDIRECTORY(tests)
ENDIF(CREATE_TESTS)


message(STATUS "Configuration successful. Type make to compile kdl_format_io")
