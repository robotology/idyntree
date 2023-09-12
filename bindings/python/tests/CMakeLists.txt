
add_custom_target(copy_python_tests_in_build ALL)
add_custom_command(TARGET copy_python_tests_in_build PRE_BUILD
                   COMMAND ${CMAKE_COMMAND} -E copy_directory
                   ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})

add_test(NAME PythonGeometryTests COMMAND ${Python3_EXECUTABLE} geometry.py)
add_test(NAME PythonJointTests COMMAND ${Python3_EXECUTABLE} joints.py)
add_test(NAME PythonHelperTests COMMAND ${Python3_EXECUTABLE} helpers.py)
add_test(NAME PythonModelLoaderTests COMMAND ${Python3_EXECUTABLE} modelloader.py)
add_test(NAME PythonExtWrenchesAndJointTorquesEstimatorUnitTest COMMAND ${Python3_EXECUTABLE} ExtWrenchesAndJointTorquesEstimatorUnitTest.py)
