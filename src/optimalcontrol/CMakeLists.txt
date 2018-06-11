cmake_minimum_required(VERSION 3.0)

project(idyntree_optimalcontrol)

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
                   include/iDynTree/TimeVaryingObject.h)
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
            src/TimeVaryingObject.cpp)

if (IDYNTREE_USES_IPOPT)
    list(APPEND OPTIMIZERS_HEADERS include/iDynTree/Optimizers/IpoptInterface.h)
    list(APPEND OPTIMIZERS_SOURCES src/IpoptInterface.cpp)
    list(APPEND LINK_LIST ${IPOPT_LIBRARIES})
    list(APPEND INCLUDE_LIST ${IPOPT_INCLUDE_DIRS})
endif()

find_package(OsqpEigen QUIET)
if (${OsqpEigen_FOUND})
    option(OC_USE_OSQP "Use OSQP as a solver in optimalcontrol." ON)
else()
    set(OC_USE_OSQP OFF)
endif()

if (OC_USE_OSQP)
    list(APPEND OPTIMIZERS_HEADERS include/iDynTree/Optimizers/OsqpInterface.h)
    list(APPEND OPTIMIZERS_SOURCES src/OsqpInterface.cpp)
    list(APPEND LINK_LIST   OsqpEigen::OsqpEigen osqp::osqp)
endif()




add_library(${libraryname} ${PUBLIC_HEADERS} ${INTEGRATORS_PUBLIC_HEADERS} ${OCSOLVERS_PUBLIC_HEADERS} ${OPTIMIZERS_HEADERS} ${SOURCES} ${OPTIMIZERS_SOURCES})
target_include_directories(${libraryname} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})
target_include_directories(${libraryname} SYSTEM PRIVATE ${INCLUDE_LIST})
target_include_directories(${libraryname} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_include_directories(${libraryname} PUBLIC $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(${libraryname} PUBLIC idyntree-core idyntree-model)
target_link_libraries(${libraryname} PRIVATE ${LINK_LIST})

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
set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)


if(IDYNTREE_COMPILE_TESTS)
  add_subdirectory(tests)
endif()
