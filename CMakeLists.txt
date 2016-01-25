cmake_minimum_required(VERSION 2.8.11)

project(iDynTree CXX)

# Disable in source build
if("${PROJECT_SOURCE_DIR}" STREQUAL "${PROJECT_BINARY_DIR}")
   message(FATAL_ERROR "In-source builds of iDynTree are not allowed. "
                       "Please remove CMakeCache.txt and the CMakeFiles/ directory, then build out-of-source. "
                       "If you really want to do an in-source build, feel free to comment out this error.")
endif("${PROJECT_SOURCE_DIR}" STREQUAL "${PROJECT_BINARY_DIR}")

set(VARS_PREFIX "iDynTree")

set(${VARS_PREFIX}_MAJOR_VERSION 0)
set(${VARS_PREFIX}_MINOR_VERSION 3)
set(${VARS_PREFIX}_PATCH_VERSION 10)
set(${VARS_PREFIX}_VERSION ${${VARS_PREFIX}_MAJOR_VERSION}.${${VARS_PREFIX}_MINOR_VERSION}.${${VARS_PREFIX}_PATCH_VERSION})

# Pick up our CMake scripts - they are all in the cmake subdirectory.
set(IDYNTREE_MODULE_DIR ${PROJECT_SOURCE_DIR}/cmake)
set(IDYNTREE_MODULE_PATH ${IDYNTREE_MODULE_DIR})
list(APPEND CMAKE_MODULE_PATH ${IDYNTREE_MODULE_PATH})

include(GNUInstallDirs)
if (CMAKE_VERSION LESS 3.0)
    # workaround for http://public.kitware.com/pipermail/cmake-developers/2014-January/009435.html
    set(CMAKE_INSTALL_LIBDIR lib)
endif()

#Use options specified in iDynTreeOptions.cmake file
include(iDynTreeOptions)

if(NOT IDYNTREE_ONLY_DOCS)
# Given an error if yarp or ICUB_MAIN dependencies are selected without KDL
if(IDYNTREE_USES_ICUB_MAIN)
    if(NOT IDYNTREE_USES_YARP OR NOT IDYNTREE_USES_KDL)
        message(FATAL_ERROR "Impossible to build ICUB_MAIN support if KDL and YARP are not installed")
    endif()
endif()

if(IDYNTREE_USES_YARP)
    if(NOT IDYNTREE_USES_KDL)
        message(FATAL_ERROR "Impossible to build YARP support if IDYNTREE_USES_KDL is not enabled")
    endif()
endif()

# Find dependecies

# Eigen is compulsory
find_package(Eigen3 REQUIRED)

if(IDYNTREE_USES_YARP)
    find_package(YARP REQUIRED)
    if (YARP_HAS_MATH_LIB)
        message(STATUS "found libYARP_math")
    else(YARP_HAS_MATH_LIB)
        message(FATAL_ERROR "YARP was found, but no libYARP_math was detected, please recompile yarp")
    endif(YARP_HAS_MATH_LIB)
endif()

if(IDYNTREE_USES_ICUB_MAIN)
    find_package(ICUB REQUIRED)
endif()

if(IDYNTREE_USES_KDL)
    include(OrocosKDLFindLogic)
    find_package(orocos_kdl REQUIRED)
    find_package(TinyXML    REQUIRED)
endif()

# An external TinyXML is compulsory if compiling
# with KDL support enabled, because KDL uses urdfdom
# that include TinyXML in its public headers.
# If KDL is not enabled, we can use also the internal
# copy of TinyXML, given that we don't include it in
# our public headers
if(NOT IDYNTREE_USES_KDL)
    find_package(TinyXML)
    # If no TinyXML is found in the system, use the internal one
    if(NOT ${TinyXML_FOUND})
        add_subdirectory(extern)
    endif()
endif()


include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/bin"
                          LIB_DIRS "${CMAKE_INSTALL_PREFIX}/lib"
                          DEPENDS IDYNTREE_ENABLE_RPATH
                          USE_LINK_PATH)

# add the actual components of the library
add_subdirectory(src)

include(InstallBasicPackageFiles)
install_basic_package_files(iDynTree VARS_PREFIX ${VARS_PREFIX}
                                     VERSION ${${VARS_PREFIX}_VERSION}
                                     COMPATIBILITY SameMajorVersion
                                     TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                     NO_CHECK_REQUIRED_COMPONENTS_MACRO)

include(AddUninstallTarget)

set(IDYNTREE_USES_LUA FALSE)
set(IDYNTREE_USES_PYTHON FALSE)
set(IDYNTREE_USES_MATLAB FALSE)

add_subdirectory(bindings)

# Add reminder to remove include_directories(SYSTEM as soon as we depend on CMake 2.8.12
if(NOT ${CMAKE_MINIMUM_REQUIRED_VERSION} VERSION_LESS 2.8.12)
    message(AUTHOR_WARNING "CMAKE_MINIMUM_REQUIRED_VERSION is now ${CMAKE_MINIMUM_REQUIRED_VERSION}. include_directories(SYSTEM ..) can be substituted by target_include_directories(SYSTEM ...).")
endif()

endif()

# add a dox target to generate doxygen documentation
add_subdirectory(doc)
