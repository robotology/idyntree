
#############################################################################
## Options for compiling supported languages.  There's nothing magical
## about this list, any SWIG-supported language should work - take a
## look at e.g. ruby code below for how to do it.

set(CREATE_PYTHON FALSE CACHE BOOL "Do you want to create the Python interface")
set(CREATE_LUA FALSE CACHE BOOL "Do you want to create the Lua interface")

find_package(SWIG REQUIRED)
include(${SWIG_USE_FILE})

set_source_files_properties(iDynTree.i PROPERTIES CPLUSPLUS ON)

get_property(IDYNTREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)
get_property(IDYNTREE_LIBRARIES GLOBAL PROPERTY ${VARS_PREFIX}_TARGETS)

include_directories(${IDYNTREE_INCLUDE_DIRS})
link_libraries(${IDYNTREE_LIBRARIES})
message(STATUS "Bindings libraries: " ${IDYNTREE_LIBRARIES})

if(CREATE_LUA)
  set(CMAKE_SWIG_FLAGS "-Wall;-module;iDynTree")
  find_package(Lua51 REQUIRED)
  set(target_name iDynTree_lua)
  include_directories(${LUA_INCLUDE_DIR})
  swig_add_module(${target_name} lua iDynTree.i)
  swig_link_libraries(${target_name} ${LUA_LIBRARY})
  set_target_properties(${target_name} PROPERTIES PREFIX "")
  set_target_properties(${target_name} PROPERTIES OUTPUT_NAME "iDynTree")
  set_target_properties(${target_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/lua)
endif(CREATE_LUA)
