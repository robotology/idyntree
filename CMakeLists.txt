cmake_minimum_required(VERSION 2.8.11)

project(iDynTree CXX)

# Disable in source build, unless Eclipse is used
if( ("${PROJECT_SOURCE_DIR}" STREQUAL "${PROJECT_BINARY_DIR}") AND
    (NOT ("${CMAKE_EXTRA_GENERATOR}" STREQUAL "Eclipse CDT4")) )
   message(FATAL_ERROR "In-source builds of iDynTree are not allowed. "
                       "Please remove CMakeCache.txt and the CMakeFiles/ directory, then build out-of-source. "
                       "If you really want to do an in-source build, feel free to comment out this error"
                       " or use Eclipse CDT4 Extra Generator, that is tipically used for in-source builds."
                       "Note that to use re-generate the project usign the Eclipse CDT4 Extra Generator"
                       "you need to remove CMakeCache.txt and CMakeFiles/ as well.")
endif()

set(VARS_PREFIX "iDynTree")

set(${VARS_PREFIX}_MAJOR_VERSION 0)
set(${VARS_PREFIX}_MINOR_VERSION 9)
set(${VARS_PREFIX}_PATCH_VERSION 0)
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
    # Find dependecies
    include(iDynTreeDependencies)

    if(IDYNTREE_USES_YARP)
        find_package(YARP REQUIRED)
        if (NOT YARP_HAS_MATH_LIB)
            message(FATAL_ERROR "YARP was found, but no libYARP_math was detected, please recompile yarp")
        endif()
    endif()

    # Add external libraries that are embedded in iDynTree
    # source tree, if necessary (by default does not adds
    # anything) feel free to check teh logic inside
    add_subdirectory(extern)

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

    add_subdirectory(bindings)

    # Add reminder to remove include_directories(SYSTEM as soon as we depend on CMake 2.8.12
    if(NOT ${CMAKE_MINIMUM_REQUIRED_VERSION} VERSION_LESS 2.8.12)
        message(AUTHOR_WARNING "CMAKE_MINIMUM_REQUIRED_VERSION is now ${CMAKE_MINIMUM_REQUIRED_VERSION}. include_directories(SYSTEM ..) can be substituted by target_include_directories(<target> SYSTEM ...).")
        message(AUTHOR_WARNING "CMAKE_MINIMUM_REQUIRED_VERSION is now ${CMAKE_MINIMUM_REQUIRED_VERSION}. We can now drop the if surrounding calls to target_compile_options")
    endif()

    if(NOT ${CMAKE_MINIMUM_REQUIRED_VERSION} VERSION_LESS 3.0)
        message(AUTHOR_WARNING "CMAKE_MINIMUM_REQUIRED_VERSION is now ${CMAKE_MINIMUM_REQUIRED_VERSION}. Multiline string are now supported,so we can have a readable matlab/octave test script.")
    endif()
endif()

# add a dox target to generate doxygen documentation
add_subdirectory(doc)
