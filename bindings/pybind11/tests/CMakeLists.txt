add_test (NAME pybind11_idyntree_test
  COMMAND ${Python3_EXECUTABLE} -B -m unittest discover -s ${CMAKE_CURRENT_SOURCE_DIR}
  WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/pybind11
)
