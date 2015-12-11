get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)
macro(add_modelio_urdf_unit_test classname)
    set(testsrc ${classname}UnitTest.cpp)
    set(testbinary ${classname}UnitTest)
    set(testname   UnitTest${classname})
    add_executable(${testbinary} ${testsrc})
    target_include_directories(${testbinary} PRIVATE ${EIGEN3_INCLUDE_DIR} ${IDYNTREE_TREE_INCLUDE_DIRS})
    target_link_libraries(${testbinary} idyntree-modelio-urdf)
    add_test(NAME ${testname} COMMAND ${testbinary})

    if(IDYNTREE_RUN_VALGRIND_TESTS)
        add_test(NAME memcheck_${testname} COMMAND ${MEMCHECK_COMMAND_COMPLETE} ./${testbinary})
    endif()

endmacro()

add_modelio_urdf_unit_test(URDFModelImport)
add_modelio_urdf_unit_test(URDFGenericSensorImport)
add_modelio_urdf_unit_test(PredictSensorsMeasurement)
