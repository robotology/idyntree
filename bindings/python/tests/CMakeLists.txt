
add_custom_target(copy_python_tests_in_build ALL)
add_custom_command(TARGET copy_python_tests_in_build PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_directory
                   ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

add_test(NAME PythonGeometryTests COMMAND ${Python3_EXECUTABLE} geometry.py)
add_test(NAME PythonJointTests COMMAND ${Python3_EXECUTABLE} joints.py)
add_test(NAME PythonHelperTests COMMAND ${Python3_EXECUTABLE} helpers.py)
add_test(NAME PythonModelLoaderTests COMMAND ${Python3_EXECUTABLE} modelloader.py)
add_test(NAME PythonExtWrenchesAndJointTorquesEstimatorUnitTest COMMAND ${Python3_EXECUTABLE} ExtWrenchesAndJointTorquesEstimatorUnitTest.py)
add_test(NAME PythonInverseKinematicsTests COMMAND ${Python3_EXECUTABLE} invkin.py)

set(IDYNTREE_PYTHON_TESTS
    PythonGeometryTests
    PythonJointTests
    PythonHelperTests
    PythonModelLoaderTests
    PythonExtWrenchesAndJointTorquesEstimatorUnitTest
    PythonInverseKinematicsTests
    )

get_filename_component(PARENT_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}" DIRECTORY)

if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.22")
  set_tests_properties(
    ${IDYNTREE_PYTHON_TESTS}
    PROPERTIES ENVIRONMENT_MODIFICATION "PYTHONPATH=path_list_append:${PARENT_BINARY_DIR}")
else()
  set_tests_properties(
    ${IDYNTREE_PYTHON_TESTS}
    PROPERTIES ENVIRONMENT
               "PYTHONPATH=${PARENT_BINARY_DIR}:$ENV{PYTHONPATH}")
endif()
