
#############################################################################
## Options for compiling supported languages.  There's nothing magical
## about this list, any SWIG-supported language should work - take a
## look at e.g. ruby code below for how to do it.

set(IDYNTREE_USES_PYTHON FALSE CACHE BOOL "Do you want to create the Python interface")
set(IDYNTREE_USES_LUA FALSE CACHE BOOL "Do you want to create the Lua interface")
set(IDYNTREE_USES_MATLAB FALSE CACHE BOOL "Do you want to create the MATLAB Interface")
set(IDYNTREE_GENERATE_MATLAB FALSE CACHE BOOL "Enable if you have the experimental version of SWIG necessary for generating the Matlab wrapper")


find_package(SWIG)
if(SWIG_FOUND)
    include(${SWIG_USE_FILE})

    set_source_files_properties(iDynTree.i PROPERTIES CPLUSPLUS ON)

    get_property(IDYNTREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)
    get_property(IDYNTREE_LIBRARIES GLOBAL PROPERTY ${VARS_PREFIX}_TARGETS)

    include_directories(${IDYNTREE_INCLUDE_DIRS})
    link_libraries(${IDYNTREE_LIBRARIES})

    if(IDYNTREE_USES_LUA)
        add_subdirectory(lua)
    endif(IDYNTREE_USES_LUA)

    if(IDYNTREE_USES_PYTHON)
        message(STATUS "using python!!")
        add_subdirectory(python)
    endif(IDYNTREE_USES_PYTHON)

    if(IDYNTREE_USES_MATLAB OR GENERATE_MATLAB)
       add_subdirectory(matlab)
    endif(IDYNTREE_USES_MATLAB OR GENERATE_MATLAB)
endif(SWIG_FOUND)

