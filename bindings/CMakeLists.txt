
#############################################################################
## Options for compiling supported languages.  There's nothing magical
## about this list, any SWIG-supported language should work - take a
## look at e.g. ruby code below for how to do it.

set(CREATE_PYTHON FALSE CACHE BOOL "Do you want to create the Python interface")
set(CREATE_LUA FALSE CACHE BOOL "Do you want to create the Lua interface")
set(CREATE_MATLAB FALSE CACHE BOOL "Do you want to create the MATLAB Interface")
set(GENERATE_MATLAB FALSE CACHE BOOL "Enable if you have the experimental version of SWIG necessary for generating the Matlab wrapper")

find_package(SWIG REQUIRED)
include(${SWIG_USE_FILE})
message(STATUS "SWIG_USE_FILE :" ${SWIG_USE_FILE})

set_source_files_properties(iDynTree.i PROPERTIES CPLUSPLUS ON)

get_property(IDYNTREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)
get_property(IDYNTREE_LIBRARIES GLOBAL PROPERTY ${VARS_PREFIX}_TARGETS)

include_directories(${IDYNTREE_INCLUDE_DIRS})
link_libraries(${IDYNTREE_LIBRARIES})

if(CREATE_LUA)
    add_subdirectory(lua)
endif(CREATE_LUA)

if(CREATE_MATLAB)
    add_subdirectory(matlab)
endif(CREATE_MATLAB)
