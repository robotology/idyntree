get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)

macro(add_icub_consistency_test testName)
    set(testsrc ${testName}ConsistencyTest.cpp)
    set(testbinary ${testName}ConsistencyTest)
    set(testname   ConsistencyTest${testName})
    add_executable(${testbinary} ${testsrc})
    target_include_directories(${testbinary} PRIVATE ${IDYNTREE_TREE_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
    target_link_libraries(${testbinary} idyntree-modelio-urdf idyntree-icub idyntree-core idyntree-kdl idyntree-model)
    add_test(NAME ${testname} COMMAND ${testbinary})

    if(IDYNTREE_RUN_VALGRIND_TESTS)
        add_test(NAME memcheck_${testname} COMMAND ${MEMCHECK_COMMAND_COMPLETE} ./${testbinary})
    endif()
endmacro()

add_icub_consistency_test(iCubExternalWrenchesEstimation)
