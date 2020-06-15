# Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

set(libraryname idyntree-optimalcontrol)


set(PUBLIC_HEADERS include/iDynTree/OptimalControl.h
                   include/iDynTree/DynamicalSystem.h
                   include/iDynTree/ControlledDynamicalSystem.h
                   include/iDynTree/LinearSystem.h
                   include/iDynTree/MultiBodySystem.h
                   include/iDynTree/Integrator.h
                   include/iDynTree/LinearMPC.h
                   include/iDynTree/MPC.h
                   include/iDynTree/OptimalControlProblem.h
                   include/iDynTree/OptimalControlSolver.h
                   include/iDynTree/SystemLineariser.h
                   include/iDynTree/Cost.h
                   include/iDynTree/Constraint.h
                   include/iDynTree/QuadraticLikeCost.h
                   include/iDynTree/QuadraticCost.h
                   include/iDynTree/L2NormCost.h
                   include/iDynTree/LinearConstraint.h
                   include/iDynTree/LinearCost.h
                   include/iDynTree/Controller.h
                   include/iDynTree/TimeRange.h
                   include/iDynTree/ConstraintsGroup.h
                   include/iDynTree/OptimizationProblem.h
                   include/iDynTree/Optimizer.h
                   include/iDynTree/TimeVaryingObject.h
                   include/iDynTree/SparsityStructure.h)
set(INTEGRATORS_PUBLIC_HEADERS include/iDynTree/Integrators/FixedStepIntegrator.h
                               include/iDynTree/Integrators/RK4.h
                               include/iDynTree/Integrators/ImplicitTrapezoidal.h
                               include/iDynTree/Integrators/ForwardEuler.h)
set(OCSOLVERS_PUBLIC_HEADERS include/iDynTree/OCSolvers/MultipleShootingSolver.h)


set(SOURCES src/DynamicalSystem.cpp
            src/ControlledDynamicalSystem.cpp
            src/LinearSystem.cpp
            src/MultiBodySystem.cpp
            src/Integrator.cpp
            src/LinearMPC.cpp
            src/MPC.cpp
            src/OptimalControlProblem.cpp
            src/OptimalControlSolver.cpp
            src/SystemLineariser.cpp
            src/MultipleShootingSolver.cpp
            src/Cost.cpp
            src/Constraint.cpp
            src/QuadraticCost.cpp
            src/QuadraticLikeCost.cpp
            src/L2NormCost.cpp
            src/LinearConstraint.cpp
            src/LinearCost.cpp
            src/Controller.cpp
            src/FixedStepIntegrator.cpp
            src/RK4.cpp
            src/ImplicitTrapezoidal.cpp
            src/TimeRange.cpp
            src/ConstraintsGroup.cpp
            src/OptimizationProblem.cpp
            src/Optimizer.cpp
            src/ForwardEuler.cpp
            src/TimeVaryingObject.cpp
            src/SparsityStructure.cpp)

list(APPEND OPTIMIZERS_HEADERS include/iDynTree/Optimizers/IpoptInterface.h)
if (IDYNTREE_USES_IPOPT)
    list(APPEND OPTIMIZERS_SOURCES src/IpoptInterface.cpp)
    list(APPEND LINK_LIST ${IPOPT_LIBRARIES})
    list(APPEND INCLUDE_LIST ${IPOPT_INCLUDE_DIRS})
else()
    list(APPEND OPTIMIZERS_SOURCES src/IpoptInterfaceNotImplemented.cpp)
endif()

list(APPEND OPTIMIZERS_HEADERS include/iDynTree/Optimizers/OsqpInterface.h)
if (IDYNTREE_USES_OSQPEIGEN)
    list(APPEND OPTIMIZERS_SOURCES src/OsqpInterface.cpp)
    list(APPEND LINK_LIST   OsqpEigen::OsqpEigen osqp::osqp)
else()
    list(APPEND OPTIMIZERS_SOURCES src/OsqpInterfaceNotImplemented.cpp)
endif()

list(APPEND OPTIMIZERS_HEADERS include/iDynTree/Optimizers/AlglibInterface.h)
if (IDYNTREE_USES_ALGLIB)
    list(APPEND OPTIMIZERS_SOURCES src/AlglibInterface.cpp)
    list(APPEND LINK_LIST ALGLIB)
else()
    list(APPEND OPTIMIZERS_SOURCES src/AlglibInterfaceNotImplemented.cpp)
endif()

list(APPEND OPTIMIZERS_HEADERS include/iDynTree/Optimizers/WorhpInterface.h)
if (IDYNTREE_USES_WORHP)
    list(APPEND OPTIMIZERS_SOURCES src/WorhpInterface.cpp)
    list(APPEND LINK_LIST WORHP::WORHP)
else()
    list(APPEND OPTIMIZERS_SOURCES src/WorhpInterfaceNotImplemented.cpp)
endif()

add_library(${libraryname} ${PUBLIC_HEADERS} ${INTEGRATORS_PUBLIC_HEADERS} ${OCSOLVERS_PUBLIC_HEADERS} ${OPTIMIZERS_HEADERS} ${SOURCES} ${OPTIMIZERS_SOURCES})
add_library(iDynTree::${libraryname} ALIAS ${libraryname})

target_include_directories(${libraryname} SYSTEM PRIVATE ${INCLUDE_LIST})
target_include_directories(${libraryname} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_include_directories(${libraryname} PUBLIC $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(${libraryname} PUBLIC idyntree-core idyntree-model)
target_link_libraries(${libraryname} PRIVATE ${LINK_LIST} Eigen3::Eigen)

if (IDYNTREE_USES_IPOPT)
    target_compile_definitions(${libraryname} PRIVATE ${IPOPT_DEFINITIONS})
endif()

# install
set_property(TARGET ${libraryname} PROPERTY PUBLIC_HEADER ${PUBLIC_HEADERS})

install(TARGETS ${libraryname}
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree)

install(FILES ${INTEGRATORS_PUBLIC_HEADERS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/Integrators)
install(FILES ${OPTIMIZERS_HEADERS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/Optimizers)
install(FILES ${OCSOLVERS_PUBLIC_HEADERS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/OCSolvers)


set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})

if(IDYNTREE_COMPILE_TESTS)
  add_subdirectory(tests)
endif()
