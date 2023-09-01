# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


macro(add_benchmark benchmarkName)
    set(testsrc ${benchmarkName}Benchmark.cpp)
    set(testbinary ${benchmarkName}Benchmark)
    add_executable(${testbinary} ${testsrc})
    target_link_libraries(${testbinary} PRIVATE idyntree-modelio idyntree-modelio-kdl idyntree-core
                                                idyntree-kdl idyntree-model idyntree-testmodels Eigen3::Eigen)
endmacro()

add_benchmark(Dynamics)
