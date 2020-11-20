if(${CMAKE_VERSION} VERSION_LESS "3.18.0")
    find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
else()
    find_package(Python3 COMPONENTS Interpreter Development.Module NumPy REQUIRED)
endif()

pybind11_add_module(pybind11_idyntree SYSTEM idyntree.cpp
                                    error_utilities.h error_utilities.cpp
                                    idyntree_core.h idyntree_core.cpp
                                    error_utilities.h error_utilities.cpp
                                    idyntree_model.h idyntree_model.cpp
                                    idyntree_sensors.h idyntree_sensors.cpp
                                    idyntree_modelio_urdf.h idyntree_modelio_urdf.cpp)

target_link_libraries(pybind11_idyntree PUBLIC idyntree-core
                                               idyntree-model
                                               idyntree-sensors
                                               idyntree-modelio-urdf)

# The generated Python dynamic module must have the same name as the pybind11
# module, i.e. `bindings`.
set_target_properties(pybind11_idyntree PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/idyntree
    OUTPUT_NAME "pybind")

# if compile tests execute also python tests
if(IDYNTREE_COMPILE_TESTS)
    add_subdirectory(tests)
endif()

# Output package is:
# idyntree
# |
# |- __init__.py (generated from main bindings CMake file).
# |
# |_ pybind.<cpython_extension>
install(TARGETS pybind11_idyntree DESTINATION ${PYTHON_INSTDIR})
