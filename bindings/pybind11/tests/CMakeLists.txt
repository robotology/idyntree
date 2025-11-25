add_test (NAME pybind11_idyntree_test
  COMMAND ${Python3_EXECUTABLE} -B -m unittest discover -s ${CMAKE_CURRENT_SOURCE_DIR}
  WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/pybind11
)
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.22")
  set_property(TEST pybind11_idyntree_test PROPERTY
    ENVIRONMENT_MODIFICATION
      "PYTHONPATH=path_list_append:${PROJECT_BINARY_DIR}/bindings"
      "PYTHONPATH=path_list_append:${PROJECT_BINARY_DIR}/bindings/pybind11"
      "PYTHONPATH=path_list_append:${PROJECT_BINARY_DIR}/bindings/python")
else()
  set_tests_properties(pybind11_idyntree_test
    PROPERTIES ENVIRONMENT
               "PYTHONPATH=${PROJECT_BINARY_DIR}/bindings:${PROJECT_BINARY_DIR}/bindings/pybind11:${PROJECT_BINARY_DIR}/bindings/python:$ENV{PYTHONPATH}")
endif()
