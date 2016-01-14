get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)

macro(add_integration_test testname)
    set(testsrc ${testname}IntegrationTest.cpp)
    set(testbinary ${testname}IntegrationTest)
    set(testtarget   IntegrationTest${testname})
    add_executable(${testbinary} ${testsrc})
    target_include_directories(${testbinary} PRIVATE ${IDYNTREE_TREE_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
    target_link_libraries(${testbinary} idyntree-core idyntree-model idyntree-modelio-urdf-kdl)
    add_test(NAME ${testtarget} COMMAND ${testbinary})

    if(IDYNTREE_RUN_VALGRIND_TESTS)
        add_test(NAME memcheck_${testtarget} COMMAND ${MEMCHECK_COMMAND_COMPLETE} ./${testbinary})
    endif()
endmacro()

macro(add_integration_exe testname)
    set(testsrc ${testname}IntegrationTest.cpp)
    set(testbinary ${testname}IntegrationTest)
    set(testtarget   IntegrationTest${testname})
    add_executable(${testbinary} ${testsrc})
    target_include_directories(${testbinary} PRIVATE ${EIGEN3_INCLUDE_DIR} ${IDYNTREE_TREE_INCLUDE_DIRS})
    target_link_libraries(${testbinary} idyntree-core idyntree-model idyntree-modelio-urdf-kdl)
endmacro()

add_integration_test(Dynamics)

# Until we fix it, add DynamicsLinearization test but don't execute it
add_integration_exe(DynamicsLinearization)
