cmake_minimum_required( VERSION 2.8 FATAL_ERROR )
project (kdl_format_io CXX)

set (KDL_FORMAT_IO_MAJOR_VERSION 0)
set (KDL_FORMAT_IO_MINOR_VERSION 0)
set (KDL_FORMAT_IO_PATCH_VERSION 1)

set (KDL_FORMAT_IO_VERSION ${KDL_FORMAT_IO_MAJOR_VERSION}.${KDL_FORMAT_IO_MINOR_VERSION}.${KDL_FORMAT_IO_PATCH_VERSION})

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

find_package(orocos_kdl REQUIRED)

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
   endif(NOT Orocos-KDL_FOUND)
endif(NOT orocos_kdl_FOUND)



include_directories(SYSTEM  ${orocos_kdl_INCLUDE_DIRS})

IF( ENABLE_URDF ) 
    find_package(TinyXML)
    IF( NOT TinyXML_FOUND )
        message("Disabling URDF support as no TinyXML was found")
        set(ENABLE_URDF FALSE)
    ELSE()
         include_directories(SYSTEM ${TinyXML_INCLUDE_DIRS})
    ENDIF()
ENDIF()

IF( ENABLE_URDF ) 
    find_package(urdfdom_headers)
    IF( NOT urdfdom_headers_CONFIG_INCLUDED )
        message("Disabling URDF support as no urdfdom_headers was found")
        set(ENABLE_URDF FALSE)
    ELSE()
        include_directories(SYSTEM ${urdfdom_headers_INCLUDE_DIRS})
    ENDIF()
ENDIF()

IF( ENABLE_URDF ) 
    find_package(urdfdom)
    IF( NOT urdfdom_CONFIG_INCLUDED )
        message("Disabling URDF support as no urdfdom was found")
        set(ENABLE_URDF FALSE)
    ELSE()
        include_directories(SYSTEM ${urdfdom_INCLUDE_DIRS})
    ENDIF()
ENDIF()

IF( ENABLE_URDF ) 
    find_package(console_bridge)
    IF( NOT ${console_bridge_CONFIG_INCLUDED} )
        message("Disabling URDF support as no console_bridge was found")
        set(ENABLE_URDF FALSE)
    ELSE()
        include_directories(SYSTEM ${console_bridge_INCLUDE_DIRS})
        link_directories(${console_bridge_LIBRARY_DIRS})    
    ENDIF()
ENDIF()

IF( ENABLE_URDF ) 
        find_package(Boost REQUIRED thread)
    IF( NOT ${Boost_FOUND} )
        message("Disabling URDF support as no Boost was found")
        set(ENABLE_URDF FALSE)
    ELSE()
        include_directories(SYSTEM ${console_bridge_INCLUDE_DIRS})
        link_directories(${console_bridge_LIBRARY_DIRS}) 
    ENDIF()
ENDIF()

IF(ENABLE_SERIALIZATION_IO)
    find_package(kdl_codyco)
    IF( NOT kdl_codyco_FOUND )
        message("Disabling Serialization as no kdl_codyco was found")
        set(ENABLE_SERIALIZATION_IO FALSE)
    ELSE()
         include_directories(SYSTEM ${kdl_codyco_INCLUDE_DIRS})
         link_directories(${kdl_codyco_LIBRARY_DIRS})
    ENDIF()
ENDIF()

include_directories(include)


                     
if(ENABLE_URDF) 
    set(URDF_SRCS src/converters/urdf_export.cpp src/converters/urdf_import.cpp)
    set(URDF_LIBS ${urdfdom_LIBRARIES} ${console_bridge_LIBRARIES} ${Boost_LIBRARIES})
endIF()

if(ENABLE_SYMORO_PAR)
    set(EXPR_PARSER_SRCS src/expression_parser/error.cpp
                     src/expression_parser/functions.cpp
                     src/expression_parser/parser.cpp
                     src/expression_parser/variablelist.cpp)
    set(SYMORO_PAR_SRCS src/converters/symoro_par_import.cpp ${EXPR_PARSER_SRCS})
    if(ENABLE_SERIALIZATION_IO)
        set(SYMORO_PAR_SRCS ${SYMORO_PAR_SRCS} src/converters/symoro_par_import_serialization.cpp)
    endif()
endif()

add_library(kdl_format_io SHARED ${URDF_SRCS} ${SYMORO_PAR_SRCS})

IF(ENABLE_SERIALIZATION_IO)
    target_link_libraries(kdl_format_io kdl_codyco ${URDF_LIBS}  ${orocos_kdl_LIBRARIES})
ELSE()
    target_link_libraries(kdl_format_io ${URDF_LIBS}  ${orocos_kdl_LIBRARIES})
ENDIF()


IF( ENABLE_URDF AND ENABLE_SYMORO_PAR )
    add_executable(par2urdf src/utils/par2urdf.cpp)
    target_link_libraries(par2urdf kdl_format_io ${URDF_LIBS} ${orocos_kdl_LIBRARIES})
ENDIF()

#add_executable(check_kdl_export src/check_kdl_export.cpp src/kdl_import.cpp src/kdl_export.cpp)
#target_link_libraries(check_kdl_export ${kdl_format_io_LIBRARIES} ${urdfdom_headers_LIBRARIES}  ${urdfdom_LIBRARIES} ${console_bridge_LIBRARIES} ${Boost_LIBRARIES} ${Orocos-KDL_LIBRARIES})


INSTALL(TARGETS kdl_format_io DESTINATION lib/)
INSTALL(DIRECTORY include/kdl_format_io DESTINATION include)



set(PKG_NAME ${PROJECT_NAME})
set(PKG_LIBRARIES kdl_format_io)
set(PKG_DEPENDS urdfdom urdfdom_headers console_bridge)
set(cmake_conf_file "${CMAKE_CURRENT_SOURCE_DIR}/cmake/kdl_format_io-config.cmake")
configure_file("${cmake_conf_file}.in" "${cmake_conf_file}" @ONLY)
install(FILES ${cmake_conf_file} DESTINATION share/${PROJECT_NAME}/cmake/ COMPONENT cmake)

# Make the package config file
if (NOT MSVC)
  set(PKG_DESC "Kinematics Dynamics Library support for various robot description formats")
  set(PKG_DEPENDS "orocos_kdl urdfdom urdfdom_headers console_bridge") # make the list separated by spaces instead of ;
  set(PKG_URDF_LIBS "-lkdl_format_io")
  set(pkg_conf_file "${CMAKE_CURRENT_SOURCE_DIR}/cmake/pkgconfig/kdl_format_io.pc")
  configure_file("${pkg_conf_file}.in" "${pkg_conf_file}" @ONLY)
  install(FILES ${pkg_conf_file} DESTINATION lib/pkgconfig/ COMPONENT pkgconfig)
endif()

option(CREATE_TESTS "Create regression tests using CMake" FALSE)

IF(CREATE_TESTS)
    enable_testing()
    ADD_SUBDIRECTORY(tests)
ENDIF(CREATE_TESTS)


message(STATUS "Configuration successful. Type make to compile kdl_format_io")
