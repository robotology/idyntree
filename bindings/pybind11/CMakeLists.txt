find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)

pybind11_add_module(pybind11_idyntree SYSTEM idyntree.cpp 
                                    idyntree_core.h idyntree_core.cpp
                                    error_utilities.h error_utilities.cpp
                                    idyntree_model.h idyntree_model.cpp
                                    idyntree_sensors.h idyntree_sensors.cpp)

target_link_libraries(pybind11_idyntree PUBLIC idyntree-core
                                               idyntree-model
                                               idyntree-sensors)

# The generated Python dynamic module must have the same name as the pybind11
# module, i.e. `bindings`.
set_target_properties(pybind11_idyntree PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/idyntree
    OUTPUT_NAME "bindings")

file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/idyntree/__init__.py "")

# if compile tests execute also python tests
if(IDYNTREE_COMPILE_TESTS)
    add_subdirectory(tests)
endif()

# install(TARGETS pybind11_idyntree
#   COMPONENT python
#   LIBRARY DESTINATION "${Python3_LIBRARY_DIRS}"
# )