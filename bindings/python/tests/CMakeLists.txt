
add_custom_target(copy_python_tests_in_build ALL)
add_custom_command(TARGET copy_python_tests_in_build PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_directory
                   ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

set(IDYNTREE_PYTHON_TEST_PYTHONPATH
    "PYTHONPATH=path_list_append:${CMAKE_BINARY_DIR}/bindings/python"
    "PYTHONPATH=path_list_append:${CMAKE_BINARY_DIR}/bindings/pybind11"
    "PYTHONPATH=path_list_append:${CMAKE_BINARY_DIR}/bindings"
    "PYTHONPATH=path_list_append:${CMAKE_BINARY_DIR}/lib/python")

add_test(NAME PythonGeometryTests COMMAND ${Python3_EXECUTABLE} geometry.py)
add_test(NAME PythonJointTests COMMAND ${Python3_EXECUTABLE} joints.py)
add_test(NAME PythonHelperTests COMMAND ${Python3_EXECUTABLE} helpers.py)
add_test(NAME PythonModelLoaderTests COMMAND ${Python3_EXECUTABLE} modelloader.py)
add_test(NAME PythonExtWrenchesAndJointTorquesEstimatorUnitTest COMMAND ${Python3_EXECUTABLE} ExtWrenchesAndJointTorquesEstimatorUnitTest.py)
add_test(NAME PythonInverseKinematicsTests COMMAND ${Python3_EXECUTABLE} invkin.py)

if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.22")
  set_tests_properties(
    PythonGeometryTests
    PythonJointTests
    PythonHelperTests
    PythonModelLoaderTests
    PythonExtWrenchesAndJointTorquesEstimatorUnitTest
    PythonInverseKinematicsTests
    PROPERTIES ENVIRONMENT_MODIFICATION "${IDYNTREE_PYTHON_TEST_PYTHONPATH}")
else()
  set_tests_properties(
    PythonGeometryTests
    PythonJointTests
    PythonHelperTests
    PythonModelLoaderTests
    PythonExtWrenchesAndJointTorquesEstimatorUnitTest
    PythonInverseKinematicsTests
    PROPERTIES ENVIRONMENT
               "PYTHONPATH=${CMAKE_BINARY_DIR}/bindings/python:${CMAKE_BINARY_DIR}/bindings/pybind11:${CMAKE_BINARY_DIR}/bindings:${CMAKE_BINARY_DIR}/lib/python:$ENV{PYTHONPATH}")
endif()
