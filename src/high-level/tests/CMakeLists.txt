macro(add_unit_test_hl classname)
    set(testsrc ${classname}UnitTest.cpp)
    set(testbinary ${classname}UnitTest)
    set(testname   UnitTest${classname})
    add_executable(${testbinary} ${testsrc})
    target_link_libraries(${testbinary} PRIVATE idyntree-core idyntree-model idyntree-high-level idyntree-modelio-urdf
                                                idyntree-testmodels Eigen3::Eigen)
    add_test(NAME ${testname} COMMAND ${testbinary})

    if(IDYNTREE_RUN_VALGRIND_TESTS)
        add_test(NAME memcheck_${testname} COMMAND ${MEMCHECK_COMMAND_COMPLETE} $<TARGET_FILE:${testbinary}>)
    endif()
endmacro()

# todo
add_unit_test_hl(KinDynComputations)
add_unit_test_hl(KinDynComputationsMatrixViewAndSpan)
