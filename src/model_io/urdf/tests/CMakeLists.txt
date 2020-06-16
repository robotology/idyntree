# Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

macro(add_modelio_urdf_unit_test classname)
    set(testsrc ${classname}UnitTest.cpp)
    set(testbinary ${classname}UnitTest)
    set(testname   UnitTest${classname})
    add_executable(${testbinary} ${testsrc})
    target_link_libraries(${testbinary} PRIVATE idyntree-modelio-urdf idyntree-testmodels Eigen3::Eigen)
    add_test(NAME ${testname} COMMAND ${testbinary})

    if(IDYNTREE_RUN_VALGRIND_TESTS)
        add_test(NAME memcheck_${testname} COMMAND ${MEMCHECK_COMMAND_COMPLETE} $<TARGET_FILE:${testbinary}>)

    endif()

endmacro()

add_modelio_urdf_unit_test(URDFModelImport)
add_modelio_urdf_unit_test(ModelCalibrationHelper)
add_modelio_urdf_unit_test(ModelExporter)
add_modelio_urdf_unit_test(URDFGenericSensorImport)
add_modelio_urdf_unit_test(PredictSensorsMeasurement)
add_modelio_urdf_unit_test(icubSensorURDF)
