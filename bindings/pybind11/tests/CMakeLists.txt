add_test (NAME pybind11_idyntree_test
  COMMAND ${Python3_EXECUTABLE} -B -m unittest discover -s ${CMAKE_CURRENT_SOURCE_DIR}
  WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/pybind11
)

get_filename_component(PARENT_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}" DIRECTORY)

if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.22")
  set_property(TEST pybind11_idyntree_test PROPERTY
    ENVIRONMENT_MODIFICATION
      "PYTHONPATH=path_list_append:${PARENT_BINARY_DIR}")
else()
  set_tests_properties(pybind11_idyntree_test
    PROPERTIES ENVIRONMENT
               "PYTHONPATH=${PARENT_BINARY_DIR}:$ENV{PYTHONPATH}")
endif()
