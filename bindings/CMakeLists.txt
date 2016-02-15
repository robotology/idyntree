
#############################################################################
## Options for compiling supported languages.  There's nothing magical
## about this list, any SWIG-supported language should work - take a
## look at e.g. ruby code below for how to do it.

set(IDYNTREE_USES_PYTHON FALSE CACHE BOOL "Do you want to create the Python interface")
set(IDYNTREE_USES_LUA FALSE CACHE BOOL "Do you want to create the Lua interface")
set(IDYNTREE_USES_MATLAB FALSE CACHE BOOL "Do you want to create the MATLAB Interface")
set(IDYNTREE_GENERATE_MATLAB FALSE CACHE BOOL "Enable if you have the experimental version of SWIG necessary for generating the Matlab wrapper")

if(IDYNTREE_USES_PYTHON OR
   IDYNTREE_USES_LUA OR
   IDYNTREE_USES_MATLAB OR
   IDYNTREE_GENERATE_MATLAB)
    if(NOT IDYNTREE_USES_KDL)
        message(FATAL_ERROR "Impossible generated bindings if KDL is not enabled")
    endif()
endif()

find_package(SWIG)
if(SWIG_FOUND)
    include(UseSWIG)

    set_source_files_properties(iDynTree.i PROPERTIES CPLUSPLUS ON)

    get_property(IDYNTREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)
    get_property(IDYNTREE_LIBRARIES GLOBAL PROPERTY ${VARS_PREFIX}_TARGETS)

    include_directories(${IDYNTREE_INCLUDE_DIRS})
    link_libraries(${IDYNTREE_LIBRARIES})

    # list all dependencies of the process of generating
    # SWIG bindings (for now just .i files included in the main
    # iDynTree.i file)
    # This will not be necessary once http://public.kitware.com/Bug/view.php?id=4147
    # is resolved
    # (not working at the moment, todo \TODO fix)
    set(IDYNTREE_SWIG_DEPENDS_I_FILES ${CMAKE_CURRENT_SOURCE_DIR}/ignore.i
                                      ${CMAKE_CURRENT_SOURCE_DIR}/macrosForTemplates.i
                                      ${CMAKE_CURRENT_SOURCE_DIR}/sensors.i)

    if(IDYNTREE_USES_LUA)
        add_subdirectory(lua)
    endif(IDYNTREE_USES_LUA)

    if(IDYNTREE_USES_PYTHON)
        add_subdirectory(python)
    endif(IDYNTREE_USES_PYTHON)

    if(IDYNTREE_USES_MATLAB OR IDYNTREE_GENERATE_MATLAB)
       add_subdirectory(matlab)
    endif(IDYNTREE_USES_MATLAB OR IDYNTREE_GENERATE_MATLAB)
endif(SWIG_FOUND)

if(IDYNTREE_USES_PYTHON OR
   IDYNTREE_USES_LUA    OR
   IDYNTREE_USES_MATLAB OR
   IDYNTREE_GENERATED_MATLAB)
   if(NOT SWIG_FOUND)
       MESSAGE(FATAL_ERROR "Swig not found, impossible to compile iDynTree bindings.")
   endif()
endif()

