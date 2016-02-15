get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)

macro(add_benchmark benchmarkName)
    set(testsrc ${benchmarkName}Benchmark.cpp)
    set(testbinary ${benchmarkName}Benchmark)
    add_executable(${testbinary} ${testsrc})
    target_include_directories(${testbinary} PRIVATE ${IDYNTREE_TREE_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
    target_link_libraries(${testbinary} idyntree-modelio-urdf-kdl idyntree-core idyntree-kdl idyntree-model)
endmacro()

add_benchmark(Dynamics)
