
add_custom_target(copy_python_tests_in_build ALL)
add_custom_command(TARGET copy_python_tests_in_build PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_directory
                   ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

add_test(NAME PythonGeometryTests COMMAND ${PYTHON_EXECUTABLE} geometry.py)
add_test(NAME PythonJointTests COMMAND ${PYTHON_EXECUTABLE} joints.py)
add_test(NAME PythonHelperTests COMMAND ${PYTHON_EXECUTABLE} helpers.py)
add_test(NAME PythonDeprecatedTests COMMAND ${PYTHON_EXECUTABLE} deprecated.py)
