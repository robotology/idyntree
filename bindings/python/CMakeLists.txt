cmake_policy(SET CMP0078 NEW)
cmake_policy(SET CMP0086 NEW)

set_source_files_properties(../iDynTree.i PROPERTIES
    CPLUSPLUS ON
    SWIG_MODULE_NAME "swig")

set(target_name iDynTree)

swig_add_library(${target_name}
    TYPE MODULE
    LANGUAGE python
    SOURCES ../iDynTree.i
    OUTFILE_DIR ${CMAKE_CURRENT_BINARY_DIR}
    OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/idyntree)

message(STATUS "Linking SWIG Python bindings against NumPy '${Python3_NumPy_VERSION}'")
target_link_libraries(${target_name} PUBLIC Python3::NumPy)

set_target_properties(${target_name} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/idyntree)

set_property(
    TARGET ${target_name}
    PROPERTY SWIG_DEPENDS python.i numpy.i)

if(WIN32)
    set_target_properties(${target_name} PROPERTIES SUFFIX ".pyd")
endif(WIN32)

# Fix for PyPy
# See https://gitlab.kitware.com/cmake/cmake/-/issues/23651
# See https://github.com/conda-forge/idyntree-feedstock/pull/44
if(Python3_INTERPRETER_ID STREQUAL "PyPy")
    execute_process(COMMAND ${Python3_EXECUTABLE}
                    -c "import sysconfig; print(sysconfig.get_config_var('EXT_SUFFIX'))"
                    OUTPUT_VARIABLE IDYNTREE_PYTHON_EXT_SUFFIX)
    string(STRIP ${IDYNTREE_PYTHON_EXT_SUFFIX} IDYNTREE_PYTHON_EXT_SUFFIX)
    set_property (TARGET ${target_name} PROPERTY SUFFIX "${IDYNTREE_PYTHON_EXT_SUFFIX}")
endif()

if(NOT MSVC)
    set_property(
        TARGET ${target_name}
        PROPERTY SWIG_COMPILE_OPTIONS -Wextra -threads)

    set_property(
        TARGET ${target_name}
        PROPERTY SWIG_GENERATED_COMPILE_OPTIONS -Wextra)
endif()

install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/idyntree/swig.py
    DESTINATION ${PYTHON_INSTDIR}
    COMPONENT python)

install(
    TARGETS ${target_name}
    DESTINATION ${PYTHON_INSTDIR}
    COMPONENT python)

install(
    DIRECTORY visualize
    DESTINATION ${PYTHON_INSTDIR}
    COMPONENT python)

add_subdirectory(scripts)

# if compile tests execute also python tests
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
