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
option(IDYNTREE_BUILD_TESTS "Compile iDynTree tests" TRUE)

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_codyco 0.0.3 REQUIRED)

if(IDYNTREE_ENABLE_URDF)
find_package(kdl_format_io REQUIRED)
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
                                            
target_link_libraries(idyntree iDyn
                                     ${GSL_LIBRARIES}
                                     ${YARP_LIBRARIES}
                                     ${kdl_codyco_LIBRARIES}
                                     ${kdl_format_io_LIBRARIES}
                                     ${Boost_LIBRARIES})

install(TARGETS idyntree
        EXPORT iDynTree
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}/iCub/iDynTree" COMPONENT dev)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS idyntree)

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