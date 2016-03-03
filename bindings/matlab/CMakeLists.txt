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
    message(STATUS "swig generated sources" ${swig_generated_sources})
  endforeach()
endmacro()

macro(SWIG_COMPILE_MODULE name language)
  SWIG_MODULE_INITIALIZE(${name} ${language})
  message("-----> ${SWIG_MODULE_${name}_REAL_NAME} " ${SWIG_MODULE_${name}_REAL_NAME})
  message(swig generated sources--> ${swig_generated_sources})
  message(other ----> ${swig_other_sources})
  add_library(${SWIG_MODULE_${name}_REAL_NAME}
              MODULE
              ${swig_generated_sources}
              ${swig_other_sources})
endmacro()


# Find MATLAB
find_package(Matlab
             REQUIRED
             MX_LIBRARY
             MAIN_PROGRAM)

include_directories(${Matlab_INCLUDE_DIRS})

# The name of the generated source named is
# defined by the variable swig_generated_file_fullname
# in the SWIG_ADD_SOURCE_TO_MODULE macro in UseSWIG.cmake
set(sourcename iDynTreeMATLAB_wrap)

# The name of the generated source should instead match the one
# used by SWIG when generating the .m file.
# See https://github.com/jaeandersson/swig/issues/44 for more details
set(mexname    iDynTreeMEX)

# Generate SWIG wrapper
if(IDYNTREE_GENERATE_MATLAB)
  # generate the wrapper
  set(swig_generated_sources)
  set_source_files_properties(../iDynTree.i PROPERTIES CPLUSPLUS ON)
  # generate files in the source directory, so we can commit it
  set(CMAKE_SWIG_OUTDIR ${CMAKE_CURRENT_SOURCE_DIR})
  #set(CMAKE_SWIG_FLAGS "-redirectoutput")
  set(SWIG_MODULE_${mexname}_EXTRA_DEPS ${IDYNTREE_SWIG_DEPENDS_I_FILES})
  swig_generate_module(${mexname} matlab ../iDynTree.i)
  set_source_files_properties(${CMAKE_CURRENT_SOURCE_DIR}/${sourcename}.cxx PROPERTIES GENERATED 1)
endif()

# use previously generated files
set(swig_generated_sources ${CMAKE_CURRENT_SOURCE_DIR}/${sourcename}.cxx)
set(swig_other_sources)

# Set the generated mex name to be iDynTreeMEX, as it the defaul one used by SWIG while generating bindings
swig_compile_module(${mexname} matlab)
swig_link_libraries(${mexname} ${Matlab_LIBRARIES} ${IDYNTREE_LIBRARIES} idyntree-core)
set_target_properties(${mexname} PROPERTIES PREFIX "" SUFFIX .${Matlab_MEX_EXTENSION})

# Install the generated front-end to ${CMAKE_INSTALL_PREFIX}/matlab_tb
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/+iDynTree DESTINATION ${CMAKE_INSTALL_PREFIX}/mex)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/SwigRef.m DESTINATION ${CMAKE_INSTALL_PREFIX}/mex)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/SwigMem.m DESTINATION ${CMAKE_INSTALL_PREFIX}/mex)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/iDynTreesetup.m DESTINATION ${CMAKE_INSTALL_PREFIX}/mex)
install(TARGETS ${mexname} DESTINATION ${CMAKE_INSTALL_PREFIX}/mex)

#On new versions of Clang, MATLAB requires C++11.
#I enable it on all Clangs
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    if(${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
        if (${CMAKE_GENERATOR} MATCHES "Xcode")
            #this should set explictly the option in xcode. Unfortunately it does not work.
            set(XCODE_ATTRIBUTE_CLANG_CXX_LANGUAGE_STANDARD "C++11")
        endif(${CMAKE_GENERATOR} MATCHES "Xcode")
    endif(${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

# if compile tests compile also matlab tests
if(IDYNTREE_COMPILE_TESTS)
    add_subdirectory(tests)
endif()
