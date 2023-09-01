# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

macro(add_unit_test classname)
    set(testsrc ${classname}UnitTest.cpp)
    set(testbinary ${classname}UnitTest)
    set(testname   UnitTest${classname})
    add_executable(${testbinary} ${testsrc})
    target_link_libraries(${testbinary} PRIVATE idyntree-core Eigen3::Eigen)
    add_test(NAME ${testname} COMMAND ${testbinary})

    if(IDYNTREE_RUN_VALGRIND_TESTS)
        add_test(NAME memcheck_${testname} COMMAND ${MEMCHECK_COMMAND_COMPLETE} $<TARGET_FILE:${testbinary}>)
    endif()
endmacro()

add_unit_test(Axis)
add_unit_test(Direction)
add_unit_test(PrivateUtils)
add_unit_test(SpatialAcc)
add_unit_test(SpatialInertia)
add_unit_test(ArticulatedBodyInertia)
add_unit_test(Twist)
add_unit_test(Wrench)
add_unit_test(ExpLog)
add_unit_test(VectorDynSize)
add_unit_test(MatrixDynSize)
add_unit_test(SparseMatrix)
add_unit_test(EigenHelpers)
add_unit_test(Rotation)
add_unit_test(EigenSparseHelpers)
add_unit_test(TransformFromMatrix4x4)
add_unit_test(CubicSpline)
add_unit_test(Span)
add_unit_test(SO3Utils)
add_unit_test(MatrixView)


# We have also some usages of the API that we want to make sure that do not compile
macro(add_compilation_error_test name)
    set(testsrc ${name}CompilationErrorTest.cpp)
    set(testbinary ${name}CompilationErrorTest)
    set(testname   CompilationErrorTest${name})

    # Inspired from  https://stackoverflow.com/questions/30155619/expected-build-failure-tests-in-cmake
    add_executable(${testbinary} ${testsrc})
    set_target_properties(${testbinary} PROPERTIES EXCLUDE_FROM_ALL TRUE
                                                   EXCLUDE_FROM_DEFAULT_BUILD TRUE)
    target_link_libraries(${testbinary} PRIVATE idyntree-core Eigen3::Eigen)

    add_test(NAME ${testname}
             COMMAND ${CMAKE_COMMAND} --build . --target ${testbinary} --config $<CONFIGURATION>
             WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
    set_tests_properties(${testname} PROPERTIES WILL_FAIL TRUE)

endmacro()

add_compilation_error_test(SpatialToEigen)
