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

target_link_libraries(${target_name} PUBLIC Python3::NumPy)

set_target_properties(${target_name} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/idyntree)

set_property(
    TARGET ${target_name}
    PROPERTY SWIG_DEPENDS python.i numpy.i)

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
    DESTINATION ${PYTHON_INSTDIR})

install(TARGETS ${target_name} DESTINATION ${PYTHON_INSTDIR})

if(WIN32)
    set_target_properties(${target_name} PROPERTIES SUFFIX ".pyd")
endif(WIN32)

# if compile tests execute also python tests
if(IDYNTREE_COMPILE_TESTS)
    add_subdirectory(tests)
endif()
