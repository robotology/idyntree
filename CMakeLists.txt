cmake_minimum_required(VERSION 2.8.11)
project(iDynTree CXX)

set(VARS_PREFIX "iDynTree")

set(${VARS_PREFIX}_MAJOR_VERSION 0)
set(${VARS_PREFIX}_MINOR_VERSION 3)
set(${VARS_PREFIX}_PATCH_VERSION 5)
set(${VARS_PREFIX}_VERSION ${${VARS_PREFIX}_MAJOR_VERSION}.${${VARS_PREFIX}_MINOR_VERSION}.${${VARS_PREFIX}_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

# Pick up our CMake scripts - they are all in the cmake subdirectory.
set(IDYNTREE_MODULE_DIR ${PROJECT_SOURCE_DIR}/cmake)
set(IDYNTREE_MODULE_PATH ${IDYNTREE_MODULE_DIR})
list(APPEND CMAKE_MODULE_PATH ${IDYNTREE_MODULE_PATH})

#Use options specified in iDynTreeOptions.cmake file
include(iDynTreeOptions)

# Find dependecies
find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
include(OrocosKDLFindLogic)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

if (YARP_HAS_MATH_LIB)
    message(STATUS "found libYARP_math")
else(YARP_HAS_MATH_LIB)
    message(FATAL_ERROR "YARP was found, but no libYARP_math was detected, please recompile yarp")
endif(YARP_HAS_MATH_LIB)

find_package(TinyXML REQUIRED)

include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/bin"
                          LIB_DIRS "${CMAKE_INSTALL_PREFIX}/lib"
                          DEPENDS IDYNTREE_ENABLE_RPATH
                          USE_LINK_PATH)

add_subdirectory(core-experimental)
add_subdirectory(core)
add_subdirectory(regressors)
add_subdirectory(model_io)
add_subdirectory(yarp)

# compile simple command line tools
add_subdirectory(tools)

include(InstallBasicPackageFiles)
install_basic_package_files(iDynTree VARS_PREFIX ${VARS_PREFIX}
                                     VERSION ${${VARS_PREFIX}_VERSION}
                                     COMPATIBILITY SameMajorVersion
                                     TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                     NO_CHECK_REQUIRED_COMPONENTS_MACRO)

include(AddUninstallTarget)

if(IDYNTREE_COMPILE_TESTS)
    add_subdirectory(tests)
endif(IDYNTREE_COMPILE_TESTS)

if(IDYNTREE_COMPILE_BINDINGS)
add_subdirectory(bindings)
endif(IDYNTREE_COMPILE_BINDINGS)