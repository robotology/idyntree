set(CMAKE_SWIG_FLAGS "-Wextra;-module;iDynTree")
find_package(Lua51 REQUIRED)
set(target_name iDynTree_lua)
include_directories(${LUA_INCLUDE_DIR})

set(SWIG_MODULE_${target_name}_EXTRA_DEPS ${IDYNTREE_SWIG_DEPENDS_I_FILES})
set_source_files_properties(../iDynTree.i PROPERTIES CPLUSPLUS ON)
swig_add_module(${target_name} lua ../iDynTree.i)
swig_link_libraries(${target_name} ${LUA_LIBRARY} ${IDYNTREE_LIBRARIES})

set_target_properties(${target_name} PROPERTIES PREFIX "")
set_target_properties(${target_name} PROPERTIES OUTPUT_NAME "iDynTree")
set_target_properties(${target_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/lua)
