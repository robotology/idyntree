pybind11_add_module(pybind11_idyntree idyntree.cpp
                                    error_utilities.h error_utilities.cpp
                                    idyntree_core.h idyntree_core.cpp
                                    error_utilities.h error_utilities.cpp
                                    idyntree_model.h idyntree_model.cpp
                                    idyntree_sensors.h idyntree_sensors.cpp
                                    idyntree_modelio_urdf.h idyntree_modelio_urdf.cpp
                                    idyntree_high_level.h idyntree_high_level.cpp)

target_link_libraries(pybind11_idyntree PUBLIC idyntree-core
                                               idyntree-model
                                               idyntree-modelio
                                               idyntree-high-level)

# The generated Python dynamic module must have the same name as the pybind11
# module, i.e. `bindings`.
set_target_properties(pybind11_idyntree PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/idyntree
    OUTPUT_NAME "pybind")

# if compile tests execute also python tests
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

# Output package is:
# idyntree
# |
# |- __init__.py (generated from main bindings CMake file).
# |
# |_ pybind.<cpython_extension>
install(
    TARGETS pybind11_idyntree
    DESTINATION ${PYTHON_INSTDIR}
    COMPONENT python)
