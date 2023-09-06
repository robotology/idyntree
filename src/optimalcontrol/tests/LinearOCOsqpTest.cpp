// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include <iDynTree/Utils.h>
#include <iDynTree/TestUtils.h>
#include <iDynTree/VectorDynSize.h>
#include <iDynTree/MatrixDynSize.h>
#include <iDynTree/LinearSystem.h>
#include <iDynTree/L2NormCost.h>
#include <iDynTree/LinearConstraint.h>
#include <iDynTree/ConstraintsGroup.h>
#include <iDynTree/TimeRange.h>
#include <iDynTree/Integrators/ForwardEuler.h>
#include <iDynTree/OCSolvers/MultipleShootingSolver.h>
#include <iDynTree/Optimizers/OsqpInterface.h>
//#include <iDynTree/Optimizers/IpoptInterface.h>
#include <iDynTree/OptimalControlProblem.h>

#include <vector>
#include <ctime>

using namespace iDynTree::optimalcontrol;
using namespace iDynTree;

int main() {
    std::shared_ptr<LinearSystem> doubleIntegrator(new LinearSystem(2, 1));

    MatrixDynSize stateMatrix(2,2), controlMatrix(2,1);
    stateMatrix.zero();
    stateMatrix(0,1) = 1.0;
    controlMatrix.zero();
    controlMatrix(1,0) = 1.0;

    ASSERT_IS_TRUE(doubleIntegrator->setStateMatrix(stateMatrix));
    ASSERT_IS_TRUE(doubleIntegrator->setControlMatrix(controlMatrix));

    std::shared_ptr<OptimalControlProblem> problem(new OptimalControlProblem());
    ASSERT_IS_TRUE(problem->setDynamicalSystemConstraint(doubleIntegrator));

    std::shared_ptr<L2NormCost> quadraticCost(new L2NormCost("normCost", 2, 1));
    ASSERT_IS_TRUE(problem->addLagrangeTerm(1.0, quadraticCost));

    double horizonLength = 1.0;
    ASSERT_IS_TRUE(problem->setTimeHorizon(0.0, horizonLength));

    iDynTree::VectorDynSize bound(1);
    bound(0) = 0.8;
    ASSERT_IS_TRUE(problem->setControlUpperBound(bound));
    bound(0) = -0.9;
    ASSERT_IS_TRUE(problem->setControlLowerBound(bound));

    std::shared_ptr<LinearConstraint> controlConstraint(new LinearConstraint(1, "linearConstraint"));
    MatrixDynSize controlConstraintMatrix(1,1);
    controlConstraintMatrix(0,0) = 1.0;
    ASSERT_IS_TRUE(controlConstraint->setControlConstraintMatrix(controlConstraintMatrix));
    iDynTree::VectorDynSize upperBound(1);
    upperBound(0) = 0.5;
    ASSERT_IS_TRUE(controlConstraint->setUpperBound(upperBound));

    std::shared_ptr<ConstraintsGroup> group(new ConstraintsGroup("group", 1));
    ASSERT_IS_TRUE(group->addConstraint(controlConstraint, iDynTree::optimalcontrol::TimeRange(0.6, 1.0)));
    ASSERT_IS_TRUE(problem->addGroupOfConstraints(group));

    MultipleShootingSolver solver(problem);


    std::shared_ptr<integrators::ForwardEuler> integrator(new integrators::ForwardEuler);
    ASSERT_IS_TRUE(solver.setIntegrator(integrator));


    std::shared_ptr<optimization::OsqpInterface> optimizer(new optimization::OsqpInterface);
    optimizer->settings().verbose = false;
    optimizer->settings().scaling = 0;

//    std::shared_ptr<optimization::IpoptInterface> optimizer(new optimization::IpoptInterface);
//    ASSERT_IS_TRUE(optimizer->setIpoptOption("print_level", 0));

    ASSERT_IS_TRUE(solver.setOptimizer(optimizer));
    ASSERT_IS_TRUE(solver.setStepSizeBounds(0.001, 0.011));
    ASSERT_IS_TRUE(solver.setControlPeriod(0.01));

    iDynTree::VectorDynSize initialState(2);
    iDynTree::getRandomVector(initialState, -2.0, 2.0);
    ASSERT_IS_TRUE(solver.setInitialState(initialState));

    clock_t initT, endT;
    initT = clock();
    ASSERT_IS_TRUE(solver.solve());
    endT = clock();

    std::vector<iDynTree::VectorDynSize> states, controls;
    ASSERT_IS_TRUE(solver.getSolution(states, controls));
    std::cerr << "Initial state: " << initialState.toString() << std::endl;
    std::cerr << "First state: " << states.front().toString() << std::endl;
    std::cerr << "Last state: " << states.back().toString() << std::endl;
    std::cerr << "First control: " << controls.front().toString() << std::endl;
    std::cerr << "Last control: " << controls.back().toString() << std::endl;
    std::cerr << "Elapsed time: " <<  static_cast<double>(endT - initT) / CLOCKS_PER_SEC * 1000.0 <<" ms."<<std::endl;

    for (int i=0; i < 5; ++i){

//        std::cerr << "------------" << std::endl;

        //iDynTree::getRandomVector(initialState, -2.0, 2.0);
        initialState = states.front();
        ASSERT_IS_TRUE(solver.setInitialState(initialState));
        ASSERT_IS_TRUE(problem->setTimeHorizon(0.0 + i*0.01, horizonLength + i*0.01));


        initT = clock();
        ASSERT_IS_TRUE(solver.solve());
        //solver.solve();
        endT = clock();


        //ASSERT_IS_TRUE(solver.getSolution(states, controls));
        solver.getSolution(states, controls);
//        std::cerr << "Initial state: " << initialState.toString() << std::endl;
//        std::cerr << "First state: " << states.front().toString() << std::endl;
//        std::cerr << "Last state: " << states.back().toString() << std::endl;
//        std::cerr << "First control: " << controls.front().toString() << std::endl;
//        std::cerr << "Last control: " << controls.back().toString() << std::endl;
        std::cerr << "Elapsed time: " <<  static_cast<double>(endT - initT) / CLOCKS_PER_SEC * 1000.0 <<" ms."<<std::endl;
    }

    return EXIT_SUCCESS;
}
