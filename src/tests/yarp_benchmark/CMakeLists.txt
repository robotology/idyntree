# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


macro(add_yarp_benchmark benchmarkName)
    set(testsrc ${benchmarkName}Benchmark.cpp)
    set(testbinary ${benchmarkName}Benchmark)
    add_executable(${testbinary} ${testsrc})
    target_link_libraries(${testbinary} PRIVATE idyntree-icub idyntree-core idyntree-model idyntree-testmodels Eigen3::Eigen YARP::YARP_sig  YARP::YARP_math)
endmacro()

add_yarp_benchmark(PseudoInverse)
