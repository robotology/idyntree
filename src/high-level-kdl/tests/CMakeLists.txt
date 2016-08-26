get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)
macro(add_unit_test_hl classname)
    set(testsrc ${classname}UnitTest.cpp)
    set(testbinary ${classname}UnitTest)
    set(testname   UnitTest${classname})
    add_executable(${testbinary} ${testsrc})
    target_include_directories(${testbinary} PRIVATE ${EIGEN3_INCLUDE_DIR} ${IDYNTREE_TREE_INCLUDE_DIRS})
    target_link_libraries(${testbinary} idyntree-high-level-kdl idyntree-yarp-kdl idyntree-yarp)
    add_test(NAME ${testname} COMMAND ${testbinary})
endmacro()

macro(add_unit_test_hl_yarp classname)
    set(testsrc ${classname}UnitTest.cpp)
    set(testbinary ${classname}UnitTest)
    set(testname   UnitTest${classname})
    add_executable(${testbinary} ${testsrc})
    target_include_directories(${testbinary} PRIVATE ${EIGEN3_INCLUDE_DIR} ${IDYNTREE_TREE_INCLUDE_DIRS})
    target_link_libraries(${testbinary} idyntree-high-level-kdl idyntree-yarp-kdl idyntree-yarp idyntree-high-level)
    add_test(NAME ${testname} COMMAND ${testbinary})
endmacro()

# todo
add_unit_test_hl(DynamicsComputations)

#
add_unit_test_hl_yarp(DynamicsComputationsDynTreeConsistency)
