cmake_minimum_required(VERSION 2.8.11)
project(iDynTree CXX)

set(VARS_PREFIX "iDynTree")

set(${VARS_PREFIX}_MAJOR_VERSION 0)
set(${VARS_PREFIX}_MINOR_VERSION 0)
set(${VARS_PREFIX}_PATCH_VERSION 1)
set(${VARS_PREFIX}_VERSION ${${VARS_PREFIX}_MAJOR_VERSION}.${${VARS_PREFIX}_MINOR_VERSION}.${${VARS_PREFIX}_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})


option(IDYNTREE_ENABLE_URDF "Compile iDynTree with URDF support" FALSE)
option(IDYNTREE_SHARED_LIBRARY "Compile iDynTree as a shared library" TRUE)
option(IDYNTREE_BUILD_TESTS "Compile iDynTree tests" FALSE)
option(IDYNTREE_DISABLE_RPATH "Disable RPATH for the library" FALSE)

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_codyco 0.0.3 REQUIRED)

if(IDYNTREE_ENABLE_URDF)
find_package(kdl_format_io REQUIRED)
find_package(TinyXML REQUIRED)
add_definitions(-DCODYCO_USES_URDFDOM)
endif()

SET(folder_source src/DynTree.cpp
                  src/yarp_kdl.cpp
                  src/iCubTree.cpp
                  src/iDyn2KDL.cpp
                  src/idyn2kdl_icub.cpp)

SET(folder_header include/iCub/iDynTree/DynTree.h
                  include/iCub/iDynTree/yarp_kdl.h
                  include/iCub/iDynTree/iDyn2KDL.h
                  include/iCub/iDynTree/iCubTree.h
                  include/iCub/iDynTree/idyn2kdl_icub.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

if(MSVC)
    message(STATUS "Building iDynTree as a shared library is not currently supported on Windows")
    set(IDYNTREE_SHARED_LIBRARY FALSE)
endif()

if(IDYNTREE_SHARED_LIBRARY)
    set(LIB_TYPE SHARED)
else()
    set(LIB_TYPE STATIC)
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

# the RPATH to be used when installing, but only if it's not a system directory
#This is in the WIKI, but I don't understand its use.
# list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
# if("${isSystemDir}" STREQUAL "-1")
#    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
# endif("${isSystemDir}" STREQUAL "-1")

if(IDYNTREE_DISABLE_RPATH)
    set(CMAKE_SKIP_BUILD_RPATH TRUE)
else(IDYNTREE_DISABLE_RPATH)
    set(CMAKE_SKIP_BUILD_RPATH FALSE)
endif(IDYNTREE_DISABLE_RPATH)

add_library(idyntree ${LIB_TYPE} ${folder_source} ${folder_header})

set_target_properties(idyntree PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                           SOVERSION ${${VARS_PREFIX}_VERSION}
                                           PUBLIC_HEADER "${folder_header}")

message("kdl_codyco_INCLUDE_DIRS = ${kdl_codyco_INCLUDE_DIRS}")
target_include_directories(idyntree PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
                                           "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>"
                                            ${skinDynLib_INCLUDE_DIRS}
                                            ${iDyn_INCLUDE_DIRS}
                                            ${GSL_INCLUDE_DIRS}
                                            ${YARP_INCLUDE_DIRS}
                                            ${kdl_codyco_INCLUDE_DIRS}
                                            ${Eigen3_INCLUDE_DIR}
                                            ${kdl_format_io_INCLUDE_DIRS})

target_link_libraries(idyntree LINK_PUBLIC  ${ICUB_LIBRARIES}
                                            ${GSL_LIBRARIES}
                                            ${YARP_LIBRARIES}
                                            ${kdl_codyco_LIBRARIES}
                                            ${kdl_format_io_LIBRARIES}
                                            ${Boost_LIBRARIES}
                                            ${TinyXML_LIBRARIES})

install(TARGETS idyntree
        EXPORT iDynTree
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}/iCub/iDynTree" COMPONENT dev)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS idyntree)

#hack to include find_project(kdl_formato_io) only
#if IDYNTREE_ENABLE_URDF is true
if(IDYNTREE_ENABLE_URDF)
    set(FIND_KDL_CODYCO_IDYNTREE_CONFIG "find_package(kdl_format_io REQUIRED)")
    set(FIND_TINYXML_IDYNTREE_CONFIG "find_package(TinyXML REQUIRED)")
endif()

include(InstallBasicPackageFiles)
install_basic_package_files(iDynTree VARS_PREFIX ${VARS_PREFIX}
                                        VERSION ${${VARS_PREFIX}_VERSION}
                                        COMPATIBILITY SameMajorVersion
                                        TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                        NO_CHECK_REQUIRED_COMPONENTS_MACRO)

include(AddUninstallTarget)

if(IDYNTREE_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif(IDYNTREE_BUILD_TESTS)
