# As matlab support is higly experimental for now in swig, we split the binding generation in two phases:
#  * Generation of the .cxx code, left to the author of the library that needs to have a recent (non-standard) swig
#  * Compilation of the .cxx, that is left to the user that compiles the library.
# For doing this we split the traditional swig_add_module macro in two macro: one for generating the wrapper and one
# for compiling it. As soon as upstream swig distributed by the distro gains Matlab support, we can drop this workaround
macro(SWIG_GENERATE_MODULE name language)
  SWIG_MODULE_INITIALIZE(${name} ${language})
  set(swig_dot_i_sources)
  set(swig_other_sources)
  foreach(it ${ARGN})
    if(${it} MATCHES ".*\\.i$")
      set(swig_dot_i_sources ${swig_dot_i_sources} "${it}")
    else()

      set(swig_other_sources ${swig_other_sources} "${it}")
    endif()
  endforeach()

  set(swig_generated_sources)
  foreach(it ${swig_dot_i_sources})
    SWIG_ADD_SOURCE_TO_MODULE(${name} swig_generated_source ${it})
    set(swig_generated_sources ${swig_generated_sources} "${swig_generated_source}")
  endforeach()
endmacro()

macro(SWIG_COMPILE_MODULE name language)
  message(STATUS ${swig_generated_sources})
  add_library(${SWIG_MODULE_${name}_REAL_NAME}
    MODULE
    ${swig_generated_sources}
    ${swig_other_sources})
endmacro()

# Find MATLAB
find_package(Matlab REQUIRED)

include_directories(${Matlab_INCLUDE_DIRS})

set(modulename iDynTreeMATLAB_wrap)

# Generate SWIG wrapper
if(GENERATE_MATLAB)
  # generate the wrapper
  set(swig_generated_sources)
  message("swig_generated_sources " ${swig_generated_sources})
  set_source_files_properties(../iDynTree.i PROPERTIES CPLUSPLUS ON)
  swig_generate_module(${modulename} matlab ../iDynTree.i)
  message("swig_generated_sources " ${swig_generated_sources})
  # copy generate files in source directory
  configure_file(${CMAKE_CURRENT_BINARY_DIR}/../${modulename}.cxx ${CMAKE_CURRENT_SOURCE_DIR}/${modulename}.cxx)
  configure_file(${CMAKE_CURRENT_BINARY_DIR}/../SwigRef.m ${CMAKE_CURRENT_SOURCE_DIR}/SwigRef.m)
  file(COPY ${CMAKE_CURRENT_BINARY_DIR}/../+iDynTree DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/)
endif()

# use previously generated files
set(swig_generated_sources ${CMAKE_CURRENT_SOURCE_DIR}/${modulename}.cxx)
set(swig_other_sources)

swig_compile_module(${modulename} matlab)
swig_link_libraries(${modulename} ${Matlab_LIBRARIES} ${IDYNTREE_LIBRARIES})
set_target_properties(${modulename} PROPERTIES PREFIX "" SUFFIX .${Matlab_MEX_EXTENSION})

# Install the generated front-end to ${CMAKE_INSTALL_PREFIX}/matlab_tb
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/+iDynTree DESTINATION ${CMAKE_INSTALL_PREFIX}/mex)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/SwigRef.m DESTINATION ${CMAKE_INSTALL_PREFIX}/mex)
install(TARGETS ${modulename} DESTINATION ${CMAKE_INSTALL_PREFIX}/mex)
