set(CMAKE_SWIG_FLAGS "-Wextra;-module;iDynTree;-threads")
set(IDYNTREE_USES_PYTHON_VERSION "" CACHE STRING "Specify python version to use" )

find_package(Python3 ${IDYNTREE_USES_PYTHON_VERSION} COMPONENTS Interpreter Development NumPy REQUIRED)
message(STATUS "Using Python: ${Python3_EXECUTABLE}")

if(NOT IDYNTREE_USES_PYTHON_VERSION)
    set (IDYNTREE_USES_PYTHON_VERSION ${Python3_VERSION})
endif()

set_source_files_properties(../iDynTree.i PROPERTIES CPLUSPLUS ON)

swig_add_library(iDynTree_python
    TYPE SHARED
    LANGUAGE python
    SOURCES ../iDynTree.i)

set(target_name ${SWIG_MODULE_iDynTree_python_REAL_NAME})

target_link_libraries(${target_name} PUBLIC Python3::Python Python3::NumPy)
set_target_properties(${target_name} PROPERTIES
    OUTPUT_NAME "_iDynTree"
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/python)

set_property(
    TARGET ${target_name}
    PROPERTY SWIG_DEPENDS
    python.i numpy.i)

# installation path is determined reliably on most platforms using distutils
execute_process(COMMAND ${Python3_EXECUTABLE}
    -c "from distutils import sysconfig; print(sysconfig.get_python_lib(1,0,prefix=''))"
    OUTPUT_VARIABLE PYTHON_INSTDIR)
string(STRIP ${PYTHON_INSTDIR} PYTHON_INSTDIR)
       install(FILES ${CMAKE_CURRENT_BINARY_DIR}/iDynTree.py DESTINATION ${PYTHON_INSTDIR} )
       install(TARGETS ${target_name} DESTINATION ${PYTHON_INSTDIR} )

if(WIN32)
    set_target_properties(${target_name} PROPERTIES SUFFIX ".pyd")
endif(WIN32)

# if compile tests execute also python tests
if(IDYNTREE_COMPILE_TESTS)
    add_subdirectory(tests)
endif()
