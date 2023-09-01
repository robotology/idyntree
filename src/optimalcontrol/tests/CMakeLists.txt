# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


macro(add_oc_test classname)
    set(testsrc ${classname}Test.cpp)
    set(testbinary ${classname}UnitTest)
    set(testname   UnitTest${classname})
    add_executable(${testbinary} ${testsrc})
    target_link_libraries(${testbinary} PRIVATE idyntree-model idyntree-optimalcontrol Eigen3::Eigen)
    add_test(NAME ${testname} COMMAND ${testbinary})

    if(IDYNTREE_RUN_VALGRIND_TESTS)
        add_test(NAME memcheck_${testname} COMMAND ${MEMCHECK_COMMAND_COMPLETE} $<TARGET_FILE:${testbinary}>)
    endif()
endmacro()

add_oc_test(ConstraintsGroup)
add_oc_test(Integrators)
add_oc_test(OCProblem)
add_oc_test(MultipleShooting)
add_oc_test(L2Norm)
if (IDYNTREE_USES_IPOPT)
    add_oc_test(IpoptInterface)
    add_oc_test(OptimalControlIpopt)
endif()

if (IDYNTREE_USES_OSQPEIGEN)
    add_oc_test(LinearOCOsqp)
endif()

if (IDYNTREE_USES_ALGLIB)
    add_oc_test(AlglibInterface)
endif()

if (IDYNTREE_USES_WORHP)
    add_oc_test(WorhpInterface)
endif()

