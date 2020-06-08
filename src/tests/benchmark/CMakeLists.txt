# Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

macro(add_benchmark benchmarkName)
    set(testsrc ${benchmarkName}Benchmark.cpp)
    set(testbinary ${benchmarkName}Benchmark)
    add_executable(${testbinary} ${testsrc})
    target_include_directories(${testbinary} PRIVATE ${EIGEN3_INCLUDE_DIR})
    FIND_PACKAGE(Boost)
    include_directories(${Boost_INCLUDE_DIR})
    target_link_libraries(${testbinary} idyntree-modelio-urdf idyntree-modelio-urdf-kdl idyntree-core idyntree-kdl idyntree-model idyntree-testmodels)
endmacro()

add_benchmark(Dynamics)
