add_test (NAME pybind11_idyntree_test
  COMMAND ${Python3_EXECUTABLE} -B -m unittest discover -s ${CMAKE_CURRENT_SOURCE_DIR}
  WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/pybind11
)
set_tests_properties(pybind11_idyntree_test
  PROPERTIES ENVIRONMENT
             "PYTHONPATH=${PROJECT_BINARY_DIR}/bindings:${PROJECT_BINARY_DIR}/bindings/pybind11:${PROJECT_BINARY_DIR}/bindings/python:$ENV{PYTHONPATH}")
