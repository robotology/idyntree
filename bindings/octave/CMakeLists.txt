set(CMAKE_SWIG_FLAGS "-Wextra;-module;iDynTree;")

set(target_name iDynTree_octave)

find_package(Octave REQUIRED)

get_filename_component(OCTAVE_INCLUDE_TOP ${OCTAVE_INCLUDE_DIR} DIRECTORY)
include_directories(SYSTEM ${OCTAVE_INCLUDE_DIRS} ${OCTAVE_INCLUDE_TOP})
set_source_files_properties(../iDynTree.i PROPERTIES CPLUSPLUS ON)
swig_add_module(${target_name} octave ../iDynTree.i)
swig_link_libraries(${target_name} ${OCTAVE_LIBRARIES})

set_target_properties(${target_name} PROPERTIES PREFIX "")
set_target_properties(${target_name} PROPERTIES OUTPUT_NAME "iDynTree")
set_target_properties(${target_name} PROPERTIES SUFFIX ".oct")
