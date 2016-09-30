set(CMAKE_SWIG_FLAGS "-Wextra;-module;iDynTree;")


set(target_name iDynTree_octave)

find_package(Octave REQUIRED)

get_filename_component(OCTAVE_INCLUDE_TOP ${OCTAVE_INCLUDE_DIR} DIRECTORY)
include_directories(SYSTEM ${OCTAVE_INCLUDE_DIRS} ${OCTAVE_INCLUDE_TOP})
set_source_files_properties(../iDynTree.i PROPERTIES CPLUSPLUS ON)

# From Octave 4.2, C++11 supports needs to be explicitly enabled to
# compile Octave binding
if( OCTAVE_VERSION_STRING VERSION_GREATER 4.1 )
    include(CheckCXXCompilerFlag)
    check_cxx_Compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11_WITH_EXPLICIT_FLAG)
    if(COMPILER_SUPPORTS_CXX11_WITH_EXPLICIT_FLAG)
        add_compile_options(-std=c++11)
    endif()
endif()

swig_add_module(${target_name} octave ../iDynTree.i)
swig_link_libraries(${target_name} ${OCTAVE_LIBRARIES})

set_target_properties(${target_name} PROPERTIES PREFIX "")
set_target_properties(${target_name} PROPERTIES OUTPUT_NAME "iDynTree")
set_target_properties(${target_name} PROPERTIES SUFFIX ".oct")

install(TARGETS ${target_name} DESTINATION ${CMAKE_INSTALL_LIBDIR}/octave)

if(IDYNTREE_COMPILE_TESTS)
    add_subdirectory(tests)
endif()