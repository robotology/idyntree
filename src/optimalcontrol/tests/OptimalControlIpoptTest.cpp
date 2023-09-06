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
#include <iDynTree/DynamicalSystem.h>
#include <iDynTree/Cost.h>
#include <iDynTree/Constraint.h>
#include <iDynTree/ConstraintsGroup.h>
#include <iDynTree/TimeRange.h>
#include <iDynTree/Integrators/ForwardEuler.h>
#include <iDynTree/OCSolvers/MultipleShootingSolver.h>
#include <iDynTree/Optimizers/IpoptInterface.h>
#include <iDynTree/OptimalControlProblem.h>

#include <vector>
#include <ctime>


class DoubleIntegrator : public iDynTree::optimalcontrol::DynamicalSystem {
public:
    DoubleIntegrator()
    :iDynTree::optimalcontrol::DynamicalSystem(2,1)
    {}

    virtual ~DoubleIntegrator() override;

    virtual bool dynamics(const iDynTree::VectorDynSize& state,
                          double time,
                          iDynTree::VectorDynSize& stateDynamics) override {
        ASSERT_IS_TRUE(state.size() == 2);
        ASSERT_IS_TRUE(stateDynamics.size() == 2);

        stateDynamics(0) = state(1);
        stateDynamics(1) = controlInput(0);

        return true;
    }

    virtual bool dynamicsStateFirstDerivative(const iDynTree::VectorDynSize& state,
                                              double time,
                                              iDynTree::MatrixDynSize& dynamicsDerivative) override {
        ASSERT_IS_TRUE(state.size() == 2);
        ASSERT_IS_TRUE(dynamicsDerivative.rows() == 2 && dynamicsDerivative.cols() == 2);
        dynamicsDerivative.zero();
        dynamicsDerivative(0,1) = 1.0;

        return true;
    }

    virtual bool dynamicsControlFirstDerivative(const iDynTree::VectorDynSize& state,
                                                double time,
                                                iDynTree::MatrixDynSize& dynamicsDerivative) override {
        ASSERT_IS_TRUE(state.size() == 2);
        ASSERT_IS_TRUE(dynamicsDerivative.rows() == 2 && dynamicsDerivative.cols() == 1);
        dynamicsDerivative.zero();
        dynamicsDerivative(1,0) = 1.0;

        return true;
    }
};
DoubleIntegrator::~DoubleIntegrator(){}

class TestCost : public iDynTree::optimalcontrol::Cost {
public:
    TestCost()
        :iDynTree::optimalcontrol::Cost("TestCost")
    {}
    ~TestCost() override;

    virtual bool costEvaluation(double time,
                                const iDynTree::VectorDynSize& state,
                                const iDynTree::VectorDynSize& control,
                                double& costValue) override {
        ASSERT_IS_TRUE(state.size() == 2);
        ASSERT_IS_TRUE(control.size() == 1);
        costValue = state(0) * state(0) + state(1) * state(1) + control(0) * control(0);
        return true;
     }

    virtual bool costFirstPartialDerivativeWRTState(double time,
                                                    const iDynTree::VectorDynSize& state,
                                                    const iDynTree::VectorDynSize& control,
                                                    iDynTree::VectorDynSize& partialDerivative) override {
        ASSERT_IS_TRUE(state.size() == 2);
        ASSERT_IS_TRUE(control.size() == 1);
        ASSERT_IS_TRUE(partialDerivative.size() == 2);
        partialDerivative(0) = 2*state(0);
        partialDerivative(1) = 2*state(1);
        return true;
    }

    virtual bool costFirstPartialDerivativeWRTControl(double time,
                                                      const iDynTree::VectorDynSize& state,
                                                      const iDynTree::VectorDynSize& control,
                                                      iDynTree::VectorDynSize& partialDerivative) override {
        ASSERT_IS_TRUE(state.size() == 2);
        ASSERT_IS_TRUE(control.size() == 1);
        ASSERT_IS_TRUE(partialDerivative.size() == 1);
        partialDerivative(0) = 2*control(0);
        return true;
    }

    virtual bool costSecondPartialDerivativeWRTState(double time,
                                                     const iDynTree::VectorDynSize& state,
                                                     const iDynTree::VectorDynSize& control,
                                                     iDynTree::MatrixDynSize& partialDerivative) override {
        ASSERT_IS_TRUE(state.size() == 2);
        ASSERT_IS_TRUE(control.size() == 1);
        ASSERT_IS_TRUE(partialDerivative.rows() == 2 && partialDerivative.cols() == 2);
        partialDerivative.zero();
        partialDerivative(0,0) = 2;
        partialDerivative(1,1) = 2;
        return true;
    }

    virtual bool costSecondPartialDerivativeWRTControl(double time,
                                                       const iDynTree::VectorDynSize& state,
                                                       const iDynTree::VectorDynSize& control,
                                                       iDynTree::MatrixDynSize& partialDerivative) override {
        ASSERT_IS_TRUE(state.size() == 2);
        ASSERT_IS_TRUE(control.size() == 1);
        ASSERT_IS_TRUE(partialDerivative.rows() == 1 && partialDerivative.cols() == 1);
        partialDerivative(0,0) = 2;
        return true;
    }


    virtual bool costSecondPartialDerivativeWRTStateControl(double time,
                                                            const iDynTree::VectorDynSize& state,
                                                            const iDynTree::VectorDynSize& control,
                                                            iDynTree::MatrixDynSize& partialDerivative) override {
        ASSERT_IS_TRUE(state.size() == 2);
        ASSERT_IS_TRUE(control.size() == 1);
        ASSERT_IS_TRUE(partialDerivative.rows() == 2 && partialDerivative.cols() == 1);
        partialDerivative.zero();
        return true;
    }
};
TestCost::~TestCost(){}

class TestConstraint : public iDynTree::optimalcontrol::Constraint {
public:
    TestConstraint()
        :iDynTree::optimalcontrol::Constraint(1, "testConstraint")
    {
        iDynTree::VectorDynSize upperBound(1);
        upperBound(0) = 0.5;
        ASSERT_IS_TRUE(setUpperBound(upperBound));
    }
    TestConstraint(const std::string& name)
        :iDynTree::optimalcontrol::Constraint(1, name)
    {
        iDynTree::VectorDynSize upperBound(1);
        upperBound(0) = 0.5;
        ASSERT_IS_TRUE(setUpperBound(upperBound));
    }
    virtual ~TestConstraint() override;

    virtual bool evaluateConstraint(double time,
                                    const iDynTree::VectorDynSize& state,
                                    const iDynTree::VectorDynSize& control,
                                    iDynTree::VectorDynSize& constraint) override {
        if (state.size() != 2)
            return false;

        if (control.size() != 1)
            return false;

        ASSERT_IS_TRUE(constraint.size() == 1);

        constraint(0) = control(0);

        return true;
    }

    virtual bool constraintJacobianWRTState(double time,
                                            const iDynTree::VectorDynSize& state,
                                            const iDynTree::VectorDynSize& control,
                                            iDynTree::MatrixDynSize& jacobian) override {
        ASSERT_IS_TRUE((jacobian.rows() == 1) && (jacobian.cols() == 2));
        jacobian.zero();
        return true;
    }

    virtual bool constraintJacobianWRTControl(double time,
                                              const iDynTree::VectorDynSize& state,
                                              const iDynTree::VectorDynSize& control,
                                              iDynTree::MatrixDynSize& jacobian) override {
        ASSERT_IS_TRUE((jacobian.rows() == 1) && (jacobian.cols() == 1));
        jacobian.zero();
        jacobian(0,0) = 1.0;

        return true;
    }

    virtual size_t expectedStateSpaceSize() const override {
        return 2;
    }

    virtual size_t expectedControlSpaceSize() const override {
        return 1;
    }
};
TestConstraint::~TestConstraint(){}

int main () {
    std::shared_ptr<iDynTree::optimalcontrol::OptimalControlProblem> problem(new iDynTree::optimalcontrol::OptimalControlProblem());
    std::shared_ptr<DoubleIntegrator> doubleIntegrator(new DoubleIntegrator());
    std::shared_ptr<TestCost> cost(new TestCost);
    std::shared_ptr<iDynTree::optimalcontrol::ConstraintsGroup> group(new iDynTree::optimalcontrol::ConstraintsGroup("group", 1));
    std::shared_ptr<iDynTree::optimalcontrol::Constraint> constraint(new TestConstraint());
    std::shared_ptr<iDynTree::optimalcontrol::ForwardEuler> integrator(new iDynTree::optimalcontrol::ForwardEuler);
    std::shared_ptr<iDynTree::optimization::IpoptInterface> optimizer(new iDynTree::optimization::IpoptInterface);
    iDynTree::optimalcontrol::MultipleShootingSolver solver(problem);
    iDynTree::VectorDynSize bound(1);


    // Problem description
    ASSERT_IS_TRUE(problem->setTimeHorizon(0.0, 1.0));
    ASSERT_IS_TRUE(problem->setDynamicalSystemConstraint(doubleIntegrator));
    ASSERT_IS_TRUE(problem->addLagrangeTerm(1.0, cost));
    bound(0) = 0.8;
    ASSERT_IS_TRUE(problem->setControlUpperBound(bound));
    bound(0) = -0.9;
    ASSERT_IS_TRUE(problem->setControlLowerBound(bound));

    ASSERT_IS_TRUE(group->addConstraint(constraint, iDynTree::optimalcontrol::TimeRange(0.6, 1.0)));
    ASSERT_IS_TRUE(problem->addGroupOfConstraints(group));

    // Multiple Shooting settings
    ASSERT_IS_TRUE(solver.setIntegrator(integrator));
    ASSERT_IS_TRUE(solver.setOptimizer(optimizer));
    ASSERT_IS_TRUE(solver.setStepSizeBounds(0.001, 0.02));
    ASSERT_IS_TRUE(solver.setControlPeriod(0.01));

    iDynTree::VectorDynSize initialState(2);
    iDynTree::getRandomVector(initialState, -2.0, 2.0);
    ASSERT_IS_TRUE(solver.setInitialState(initialState));


    // Optimizer settings
    //ASSERT_IS_TRUE(optimizer->setIpoptOption("linear_solver", "ma27"));
    ASSERT_IS_TRUE(optimizer->setIpoptOption("print_level", 0));

    optimizer->useApproximatedHessians();

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
        ASSERT_IS_TRUE(problem->setTimeHorizon(0.0 + i*0.01, 1.0 + i*0.01));


        initT = clock();
        ASSERT_IS_TRUE(solver.solve());
        endT = clock();


        ASSERT_IS_TRUE(solver.getSolution(states, controls));
//        std::cerr << "Initial state: " << initialState.toString() << std::endl;
//        std::cerr << "First state: " << states.front().toString() << std::endl;
//        std::cerr << "Last state: " << states.back().toString() << std::endl;
//        std::cerr << "First control: " << controls.front().toString() << std::endl;
//        std::cerr << "Last control: " << controls.back().toString() << std::endl;
//        std::cerr << "Elapsed time: " <<  static_cast<double>(endT - initT) / CLOCKS_PER_SEC * 1000.0 <<" ms."<<std::endl;
    }

    return EXIT_SUCCESS;
}
