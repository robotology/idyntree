cmake_minimum_required(VERSION 3.16)
project(iDynTreeBindings)

# Detect if we are doing a standalone build of the bindings, using an external iDynTree
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(IDYNTREE_BINDINGS_BUILD_STANDALONE TRUE)
else()
  set(IDYNTREE_BINDINGS_BUILD_STANDALONE FALSE)
endif()

if(IDYNTREE_BINDINGS_BUILD_STANDALONE)
  # Add CMake helpers
  list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../cmake)
  find_package(iDynTree REQUIRED)
  set(IDYNTREE_LIBRARIES ${iDynTree_LIBRARIES})
  # Set variables that are set in extern
  get_filename_component(PARENT_DIR ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
  set(IDYNTREE_INTERNAL_MOXUNIT_PATH "${PARENT_DIR}/extern/MOxUnit/MOxUnit")
  set(IDYNTREE_INTERNAL_MESH2TRI_PATH "${PARENT_DIR}/extern/mesh2tri")
else()
  get_property(IDYNTREE_LIBRARIES GLOBAL PROPERTY ${VARS_PREFIX}_TARGETS)
endif()


#############################################################################
## Options for compiling supported languages.  There's nothing magical
## about this list, any SWIG-supported language should work - take a
## look at e.g. ruby code below for how to do it.
option(IDYNTREE_USES_PYTHON "Do you want to create the Python bindings" FALSE)
option(IDYNTREE_USES_LUA "Do you want to create the Lua bindings" FALSE)
option(IDYNTREE_USES_MATLAB "Do you want to create the MATLAB bindings" FALSE)
option(IDYNTREE_USES_OCTAVE "Do you want to create the OCTAVE bindings" FALSE)
option(IDYNTREE_GENERATE_MATLAB "Enable if you have the experimental version of SWIG necessary for generating the Matlab wrapper" FALSE)
option(IDYNTREE_DETECT_ACTIVE_PYTHON_SITEPACKAGES "Do you want iDynTree to detect and install in the active site-package directory? (it could be a system dir)" FALSE)

find_package(SWIG)

if(IDYNTREE_USES_PYTHON OR IDYNTREE_USES_PYTHON_PYBIND11)
    if(${CMAKE_VERSION} VERSION_LESS "3.18.0")
        set(IDYNTREE_SEARCHED_Python_Development_Component "Development")
    else()
        set(IDYNTREE_SEARCHED_Python_Development_Component "Development.Module")
    endif()
    find_package(Python3 COMPONENTS Interpreter ${IDYNTREE_SEARCHED_Python_Development_Component} NumPy REQUIRED)

    if(NOT DEFINED IDYNTREE_PYTHON_INSTALL_DIR)
       if(IDYNTREE_DETECT_ACTIVE_PYTHON_SITEPACKAGES)
           set(IDYNTREE_PYTHON_INSTALL_DIR ${Python3_SITELIB})
       else()
           if(Python3_VERSION VERSION_GREATER_EQUAL 3.12)
               execute_process(COMMAND ${Python3_EXECUTABLE} -c "import os;import sysconfig;relative_site_packages = sysconfig.get_path('purelib').replace(sysconfig.get_path('data'), '').lstrip(os.path.sep);print(relative_site_packages)"
                               OUTPUT_VARIABLE _PYTHON_INSTDIR)
           else()
               execute_process(COMMAND ${Python3_EXECUTABLE} -c "from distutils import sysconfig; print(sysconfig.get_python_lib(1,0,prefix=''))"
                               OUTPUT_VARIABLE _PYTHON_INSTDIR)
           endif()
           string(STRIP ${_PYTHON_INSTDIR} IDYNTREE_PYTHON_INSTALL_DIR)
       endif()
    endif()
    if(IDYNTREE_PACKAGE_FOR_PYPI)
        set(PYTHON_INSTDIR ${CMAKE_INSTALL_PREFIX})
    else()
        set(PYTHON_INSTDIR ${IDYNTREE_PYTHON_INSTALL_DIR}/idyntree)
    endif()
    file(TO_CMAKE_PATH "${PYTHON_INSTDIR}" PYTHON_INSTDIR)
endif()

# It is possible to compile matlab/octave bindings without using SWIG
if(SWIG_FOUND OR IDYNTREE_USES_MATLAB OR IDYNTREE_USES_OCTAVE)
    if(EXISTS ${SWIG_USE_FILE})
        include(${SWIG_USE_FILE})
    else()
        include(UseSWIG)
    endif()

    set_source_files_properties(iDynTree.i PROPERTIES CPLUSPLUS ON)

    # From https://github.com/robotology/yarp/blob/v3.3.0/bindings/CMakeLists.txt#L49
    foreach(_lib IN LISTS IDYNTREE_LIBRARIES)
        get_property(_include_dirs TARGET ${_lib} PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
        foreach(_dir IN LISTS _include_dirs)
            if("${_dir}" MATCHES "\$<BUILD_INTERFACE:(.+)>$")
                include_directories("${CMAKE_MATCH_1}")
            elseif("${_dir}" MATCHES "\$<INSTALL_INTERFACE:(.+)>$")
                # Nothing to do
            else()
                include_directories(${_dir})
            endif()
        endforeach()
    endforeach()

    link_libraries(${IDYNTREE_LIBRARIES})

    if(NOT IDYNTREE_BINDINGS_BUILD_STANDALONE)
      # Remove the deprecation warnings because by definition we always build bindings also of deprecated modules
      idyntree_disable_deprecation_warnings()
    endif()

    # list all dependencies of the process of generating
    # SWIG bindings (for now just .i files included in the main
    # iDynTree.i file)
    # This will not be necessary once http://public.kitware.com/Bug/view.php?id=4147
    # is resolved
    # (not working at the moment, todo \TODO fix)
    set(IDYNTREE_SWIG_DEPENDS_I_FILES ${CMAKE_CURRENT_SOURCE_DIR}/ignore.i
                                      ${CMAKE_CURRENT_SOURCE_DIR}/sensors.i)

    if(IDYNTREE_USES_LUA)
        add_subdirectory(lua)
    endif(IDYNTREE_USES_LUA)

    if(IDYNTREE_USES_PYTHON)
        add_subdirectory(python)
    endif(IDYNTREE_USES_PYTHON)

    if(IDYNTREE_USES_MATLAB OR IDYNTREE_GENERATE_MATLAB OR IDYNTREE_USES_OCTAVE)
        add_subdirectory(matlab)
    endif()
endif()

if(IDYNTREE_USES_PYTHON OR
   IDYNTREE_USES_LUA    OR
   IDYNTREE_GENERATED_MATLAB)
   if(NOT SWIG_FOUND)
       MESSAGE(FATAL_ERROR "Swig not found, impossible to compile or generate iDynTree bindings.")
   endif()
endif()


option(IDYNTREE_USES_PYTHON_PYBIND11 "Create iDynTree Python bindings using pybind11" FALSE)
if (IDYNTREE_USES_PYTHON_PYBIND11)
    find_package(pybind11 REQUIRED)
    add_subdirectory(pybind11)
endif()

if (IDYNTREE_USES_PYTHON_PYBIND11 OR IDYNTREE_USES_PYTHON)
    set(NEW_LINE "\n")

    # Install main __init__.py
    # Clear the file first if it exists.
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/__init__.py "")
    # If pybind is enabled, add the corresponding import.
    if (${IDYNTREE_USES_PYTHON_PYBIND11})
        file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/__init__.py "from . import pybind${NEW_LINE}")
    endif()
    # If SWIG is enabled, add the corresponding import.
    if (${IDYNTREE_USES_PYTHON})
        file(APPEND "${CMAKE_CURRENT_BINARY_DIR}/__init__.py" "from . import swig${NEW_LINE}from . import visualize${NEW_LINE}")
        # Create also an alias from swig to the old `bindings` import.
        file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/bindings.py" "from .swig import *${NEW_LINE}")
        install(
            FILES "${CMAKE_CURRENT_BINARY_DIR}/bindings.py"
            DESTINATION ${PYTHON_INSTDIR}
            COMPONENT python)
        # Add the alias in the __init__.py
        file(APPEND "${CMAKE_CURRENT_BINARY_DIR}/__init__.py" "from . import bindings${NEW_LINE}")
    endif()
    install(
        FILES "${CMAKE_CURRENT_BINARY_DIR}/__init__.py"
        DESTINATION ${PYTHON_INSTDIR}
        COMPONENT python)

    # Install METADATA file to make pip aware of the idyntree Python package
    option(IDYNTREE_PYTHON_PIP_METADATA_INSTALL "Use CMake to install Python pip metadata. Set to false if someone else install it." TRUE)
    mark_as_advanced(IDYNTREE_PYTHON_PIP_METADATA_INSTALL)
    set(IDYNTREE_PYTHON_PIP_METADATA_INSTALLER "cmake" CACHE STRING "Specify the string to identify the pip Installer. Default: cmake, change this if you are using another tool.")
    if(IDYNTREE_PYTHON_PIP_METADATA_INSTALL)
        get_filename_component(PYTHON_METADATA_PARENT_DIR ${PYTHON_INSTDIR} DIRECTORY)
        file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/METADATA "")
        file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/METADATA "Metadata-Version: 2.1${NEW_LINE}")
        file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/METADATA "Name: idyntree${NEW_LINE}")
        file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/METADATA "Version: ${iDynTree_VERSION}${NEW_LINE}")
        file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/INSTALLER "${IDYNTREE_PYTHON_PIP_METADATA_INSTALLER}${NEW_LINE}")
        install(
            FILES "${CMAKE_CURRENT_BINARY_DIR}/METADATA" "${CMAKE_CURRENT_BINARY_DIR}/INSTALLER"
            DESTINATION ${PYTHON_METADATA_PARENT_DIR}/idyntree-${iDynTree_VERSION}.dist-info
            COMPONENT python)
    endif()

endif()
