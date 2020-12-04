
#############################################################################
## Options for compiling supported languages.  There's nothing magical
## about this list, any SWIG-supported language should work - take a
## look at e.g. ruby code below for how to do it.

option(IDYNTREE_USES_PYTHON "Do you want to create the Python bindings" FALSE)
option(IDYNTREE_USES_LUA "Do you want to create the Lua bindings" FALSE)
option(IDYNTREE_USES_MATLAB "Do you want to create the MATLAB bindings" FALSE)
option(IDYNTREE_USES_OCTAVE "Do you want to create the OCTAVE bindings" FALSE)
option(IDYNTREE_GENERATE_MATLAB "Enable if you have the experimental version of SWIG necessary for generating the Matlab wrapper" FALSE)

find_package(SWIG)

# Setup installation directory for Python bindings.
if (IDYNTREE_USES_PYTHON_PYBIND11 OR IDYNTREE_USES_PYTHON)
    if(IDYNTREE_PACKAGE_FOR_PYPI)
        set(PYTHON_INSTDIR ${CMAKE_INSTALL_PREFIX})
    else()
        set(PYTHON_INSTDIR ${IDYNTREE_PYTHON_INSTALL_DIR}/idyntree)
    endif()
endif()

# It is possible to compile matlab/octave bindings without using SWIG
if(SWIG_FOUND OR IDYNTREE_USES_MATLAB OR IDYNTREE_USES_OCTAVE)
    if(EXISTS ${SWIG_USE_FILE})
        include(${SWIG_USE_FILE})
    else()
        include(UseSWIG)
    endif()

    set_source_files_properties(iDynTree.i PROPERTIES CPLUSPLUS ON)

    get_property(IDYNTREE_LIBRARIES GLOBAL PROPERTY ${VARS_PREFIX}_TARGETS)

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

    # Remove the deprecation warnings because by definition we always build bindings also of deprecated modules
    idyntree_disable_deprecation_warnings()

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
find_package(pybind11 QUIET)
if (IDYNTREE_USES_PYTHON_PYBIND11)
    if (${pybind11_FOUND})
        add_subdirectory(pybind11)
    else()
       MESSAGE(FATAL_ERROR "pybind11 not found, impossible to generate iDynTree pybind11 bindings.")
    endif()
endif()

# Install main __init__.py file.
if (IDYNTREE_USES_PYTHON_PYBIND11 OR IDYNTREE_USES_PYTHON)
    if (WIN32)
        set(NEW_LINE "\n\r")
    else()
        set(NEW_LINE "\n")
    endif()
    # Clear the file first if it exists.
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/__init__.py "")
    # If pybind is enabled, add the corresponding import.
    if (${IDYNTREE_USES_PYTHON_PYBIND11})
        file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/__init__.py "from . import pybind${NEW_LINE}")
    endif()
    # If SWIG is enabled, add the corresponding import.
    if (${IDYNTREE_USES_PYTHON})
        file(APPEND "${CMAKE_CURRENT_BINARY_DIR}/__init__.py" "from . import swig${NEW_LINE}")
        # Create also an alias from swig to the old `bindings` import.
        file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/bindings.py" "from .swig import *${NEW_LINE}")
        install(FILES "${CMAKE_CURRENT_BINARY_DIR}/bindings.py"
                DESTINATION ${PYTHON_INSTDIR})
        # Add the alias in the __init__.py
        file(APPEND "${CMAKE_CURRENT_BINARY_DIR}/__init__.py" "from . import bindings${NEW_LINE}")
    endif()
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/__init__.py"
             DESTINATION ${PYTHON_INSTDIR})
endif()
