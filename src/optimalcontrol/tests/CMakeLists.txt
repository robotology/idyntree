# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

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

