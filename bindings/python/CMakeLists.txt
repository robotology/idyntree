set(CMAKE_SWIG_FLAGS "-Wall;-module;iDynTree;-threads")

set(IDYNTREE_USES_PYTHON_VERSION "" CACHE STRING "Specify python version to use" )

find_package(PythonInterp ${IDYNTREE_USES_PYTHON_VERSION} REQUIRED)
set(IDYNTREE_USES_PYTHON_VERSION_ARGS)
  if(NOT IDYNTREE_USES_PYTHON_VERSION)
     set (IDYNTREE_USES_PYTHON_VERSION ${PYTHON_VERSION_STRING})
  endif()

find_package(PythonLibs ${IDYNTREE_USES_PYTHON_VERSION} EXACT)
include_directories(${PYTHON_INCLUDE_PATH})
link_libraries(${PYTHON_LIBRARY})

## in Windows it seems necessary to declare explictly the link_directory
if(WIN32)
  get_filename_component(PYTHON_DIR ${PYTHON_LIBRARY} PATH)
  link_directories(${PYTHON_DIR})
endif()

set(target_name iDynTree_python)


set_source_files_properties(../iDynTree.i PROPERTIES CPLUSPLUS ON)
swig_add_module(${target_name} python ../iDynTree.i)
set_target_properties(_${target_name} PROPERTIES OUTPUT_NAME "_iDynTree")
set_target_properties(_${target_name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/python)

# installation path is determined reliably on most platforms using distutils
execute_process(COMMAND ${PYTHON_EXECUTABLE}
    -c "from distutils import sysconfig; print(sysconfig.get_python_lib(1,0,prefix=''))"
    OUTPUT_VARIABLE PYTHON_INSTDIR OUTPUT_STRIP_TRAILING_WHITESPACE )
       install(FILES ${CMAKE_CURRENT_BINARY_DIR}/iDynTree.py DESTINATION ${PYTHON_INSTDIR} )
       install(TARGETS _${target_name} DESTINATION ${PYTHON_INSTDIR} )

if(WIN32)
    set_target_properties(_${target_name} PROPERTIES SUFFIX ".pyd")
endif(WIN32)