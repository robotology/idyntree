get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)

macro(add_yarp_kdl_consistency_test testName)
    set(testsrc ${testName}Consistency.cpp)
    set(testbinary ${testName}ConsistencyTest)
    set(testname   ${testName}ConsistencyTest)
    add_executable(${testbinary} ${testsrc})
    target_include_directories(${testbinary} PRIVATE ${EIGEN3_INCLUDE_DIR} ${IDYNTREE_TREE_INCLUDE_DIRS})
    target_link_libraries(${testbinary}  idyntree-modelio-urdf-kdl idyntree-yarp-kdl idyntree-icub idyntree-core idyntree-kdl idyntree-model ${YARP_LIBRARIES} ${orocos_kdl_LIBRARIES})
    add_test(NAME ${testname} COMMAND ${testbinary})

    if(IDYNTREE_RUN_VALGRIND_TESTS)
        add_test(NAME memcheck_${testname} COMMAND ${MEMCHECK_COMMAND_COMPLETE} ./${testbinary})
    endif()
endmacro()

add_yarp_kdl_consistency_test(RPY)
add_yarp_kdl_consistency_test(Quaternion)