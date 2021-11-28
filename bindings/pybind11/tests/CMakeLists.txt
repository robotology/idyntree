pybind11_add_module(test_idyntree_vector_casters test_idyntree_vector_casters.cpp)
target_link_libraries(test_idyntree_vector_casters PUBLIC idyntree-pybind11)

# The generated Python dynamic module must have the same name as the pybind11
# module, i.e. `bindings`.
set_target_properties (test_idyntree_vector_casters PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  OUTPUT_NAME "vector_casters"
)

add_test (NAME pybind11_idyntree_test
  COMMAND ${Python3_EXECUTABLE} -m unittest discover -s ${CMAKE_CURRENT_SOURCE_DIR}
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/.."
)
