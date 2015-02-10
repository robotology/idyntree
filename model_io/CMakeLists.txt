# Copyright: (C) 2015 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(iDynTree_ModelIO CXX)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

#define the support for specific file formats
#by default they are all on, they are disabled if the necessary dependencies were not found

## URDF file format support
option(ENABLE_URDF "Enable support for URDF input/output" TRUE)

## SYMORO par file format support
option(ENABLE_SYMORO_PAR "Enable support for SYMORO par input" TRUE)


option(KDL_FORMAT_IO_ENABLE_RPATH "Enable RPATH for the library" TRUE)
mark_as_advanced(KDL_FORMAT_IO_ENABLE_RPATH)

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

include_directories(SYSTEM ${kdl_codyco_INCLUDE_DIRS})
set(KDL_FORMAT_IO_INCLUDE_DIRS ${KDL_FORMAT_IO_INCLUDE_DIRS} ${kdl_codyco_INCLUDE_DIRS})
link_directories(${kdl_codyco_LIBRARY_DIRS})

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

if(IDYNTREE_USE_ICUB_MAIN)
  set(IKIN_HPPS include/kdl_format_io/iKin_export.hpp)
  set(IKIN_SRCS src/converters/iKin_export.cpp)
endif()


set(KDL_FORMAT_IO_HPPS ${SYMORO_PAR_HPPS} ${URDF_HPPS} ${IKIN_HPPS})

if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
    set(LIB_TYPE STATIC)
else()
    set(LIB_TYPE SHARED)
endif()

# share headers with all iDynTree targes
set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include")
get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)

include_directories(${IDYNTREE_TREE_INCLUDE_DIRS})



add_library(idyntree-model-io ${LIB_TYPE} ${URDF_SRCS} ${SYMORO_PAR_SRCS} ${KDL_FORMAT_IO_HPPS} ${IKIN_SRCS})

target_link_libraries(idyntree-model-io idyntree-core ${TinyXML_LIBRARIES} ${URDF_LIBS}  ${orocos_kdl_LIBRARIES} iKin)
set(KDL_FORMAT_IO_LIBRARIES idyntree-core ${URDF_LIBS} ${orocos_kdl_LIBRARIES})

if(IDYNTREE_USE_ICUB_MAIN)
    find_package(YARP REQUIRED)
    find_package(ICUB REQUIRED)
    INCLUDE_DIRECTORIES(${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
    target_link_libraries(idyntree-model-io idyntree-core ${YARP_LIBRARIES} iKin ${ICUB_LIBRARIES} ${URDF_LIBS} ${TinyXML_LIBRARIES} ${orocos_kdl_LIBRARIES})
endif()

#add_executable(check_kdl_export src/check_kdl_export.cpp src/kdl_import.cpp src/kdl_export.cpp)
#target_link_libraries(check_kdl_export ${kdl_format_io_LIBRARIES} ${urdfdom_headers_LIBRARIES}  ${urdfdom_LIBRARIES} ${console_bridge_LIBRARIES} ${Boost_LIBRARIES} ${Orocos-KDL_LIBRARIES})

set_target_properties( idyntree-model-io PROPERTIES
VERSION "${KDL_FORMAT_IO_VERSION}"
  COMPILE_FLAGS "${CMAKE_CXX_FLAGS_ADD}"
  PUBLIC_HEADER "${KDL_FORMAT_IO_HPPS}"
  )

if(${CMAKE_MINIMUM_REQUIRED_VERSION} VERSION_GREATER "2.8.12")
  message(AUTHOR_WARNING "CMAKE_MINIMUM_REQUIRED_VERSION is now ${CMAKE_MINIMUM_REQUIRED_VERSION}. This check can be removed.")
endif()
if(CMAKE_VERSION VERSION_LESS 2.8.12)
  if(NOT KDL_FORMAT_IO_DISABLE_RPATH)
      set_target_properties(idyntree-model-io PROPERTIES
          INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
      )
  endif(NOT KDL_FORMAT_IO_DISABLE_RPATH)
endif(CMAKE_VERSION VERSION_LESS 2.8.12)


install(TARGETS idyntree-model-io
        EXPORT iDynTree
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}/kdl_format_io" COMPONENT dev)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS idyntree-model-io)


if(IDYNTREE_COMPILE_TESTS)
    add_subdirectory(tests)
endif(IDYNTREE_COMPILE_TESTS)

