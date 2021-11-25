set(libraryname idyntree-pybind11)

add_library(${libraryname} INTERFACE)
target_link_libraries(${libraryname} INTERFACE idyntree-core)

# Specify include directories for both compilation and installation process.
# The $<INSTALL_PREFIX> generator expression is useful to ensure to create
# relocatable configuration files, see https://cmake.org/cmake/help/latest/manual/cmake-packages.7.html
# creating-relocatable-packages
target_include_directories(${libraryname} INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

# Specify installation targets, typology and destination folders.
install(TARGETS        ${libraryname}
  EXPORT               ${PROJECT_NAME})

install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/iDynTree/pybind11/VectorCasters.h"
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/pybind11")

add_library(iDynTree::${libraryname} ALIAS ${libraryname})
set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})

pybind11_add_module(pybind11_idyntree SYSTEM idyntree.cpp
                                    error_utilities.h error_utilities.cpp
                                    idyntree_core.h idyntree_core.cpp
                                    error_utilities.h error_utilities.cpp
                                    idyntree_model.h idyntree_model.cpp
                                    idyntree_sensors.h idyntree_sensors.cpp
                                    idyntree_modelio_urdf.h idyntree_modelio_urdf.cpp
                                    )

target_link_libraries(pybind11_idyntree PUBLIC idyntree-core
                                               idyntree-model
                                               idyntree-sensors
                                               idyntree-modelio
                                               idyntree-pybind11)

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
install(
    TARGETS pybind11_idyntree
    DESTINATION ${PYTHON_INSTDIR}
    COMPONENT python)
