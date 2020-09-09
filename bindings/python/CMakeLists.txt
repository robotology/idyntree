find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
message(STATUS "Using Python: ${Python3_EXECUTABLE}")

cmake_policy(SET CMP0078 NEW)
cmake_policy(SET CMP0086 NEW)

set_source_files_properties(../iDynTree.i PROPERTIES CPLUSPLUS ON)

set(target_name iDynTree)

swig_add_library(${target_name}
    TYPE SHARED
    LANGUAGE python
    SOURCES ../iDynTree.i
    OUTFILE_DIR ${CMAKE_CURRENT_BINARY_DIR}
    OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/idyntree)

target_link_libraries(${target_name} PUBLIC Python3::Python Python3::NumPy)

set_target_properties(${target_name} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/idyntree)

set_property(
    TARGET ${target_name}
    PROPERTY SWIG_DEPENDS python.i numpy.i)

set_property(
    TARGET ${target_name}
    PROPERTY SWIG_COMPILE_OPTIONS -Wextra -threads)

set_property(
    TARGET ${target_name}
    PROPERTY SWIG_GENERATED_COMPILE_OPTIONS -Wextra)

# Add a simple __init__.py that loads the bindings and allows using idyntree.iDynTree
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/idyntree/__init__.py "from . import iDynTree")

if(IDYNTREE_PACKAGE_FOR_PYPI)
    set(PYTHON_INSTDIR ${CMAKE_INSTALL_PREFIX})
else()
    # installation path is determined reliably on most platforms using distutils
    execute_process(COMMAND ${Python3_EXECUTABLE}
        -c "from distutils import sysconfig; print(sysconfig.get_python_lib(1,0,prefix=''))"
        OUTPUT_VARIABLE PYTHON_INSTDIR_PREFIX)
    string(STRIP ${PYTHON_INSTDIR_PREFIX} PYTHON_INSTDIR_PREFIX)
    set(PYTHON_INSTDIR ${PYTHON_INSTDIR_PREFIX}/idyntree)
endif()

install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/idyntree/__init__.py
          ${CMAKE_CURRENT_BINARY_DIR}/idyntree/iDynTree.py
    DESTINATION ${PYTHON_INSTDIR})

install(TARGETS ${target_name} DESTINATION ${PYTHON_INSTDIR})

if(WIN32)
    set_target_properties(${target_name} PROPERTIES SUFFIX ".pyd")
endif(WIN32)

# if compile tests execute also python tests
if(IDYNTREE_COMPILE_TESTS)
    add_subdirectory(tests)
endif()
