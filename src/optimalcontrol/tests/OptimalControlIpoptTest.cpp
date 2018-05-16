#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/DynamicalSystem.h>
#include <iDynTree/Cost.h>
#include <iDynTree/Constraint.h>
#include <iDynTree/ConstraintsGroup.h>
#include <iDynTree/TimeRange.h>
#include <iDynTree/Integrators/ForwardEuler.h>
#include <iDynTree/OCSolvers/MultipleShootingSolver.h>
#include <iDynTree/Optimizers/IpoptInterface.h>
#include <iDynTree/OptimalControlProblem.h>

#include <cassert>
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
        assert(state.size() == 2);
        assert(stateDynamics.size() == 2);

        stateDynamics(0) = state(1);
        stateDynamics(1) = controlInput(0);

        return true;
    }

    virtual bool dynamicsStateFirstDerivative(const iDynTree::VectorDynSize& state,
                                              double time,
                                              iDynTree::MatrixDynSize& dynamicsDerivative) override {
        assert(state.size() == 2);
        assert(dynamicsDerivative.rows() == 2 && dynamicsDerivative.cols() == 2);
        dynamicsDerivative.zero();
        dynamicsDerivative(0,1) = 1.0;

        return true;
    }

    virtual bool dynamicsControlFirstDerivative(const iDynTree::VectorDynSize& state,
                                                double time,
                                                iDynTree::MatrixDynSize& dynamicsDerivative) override {
        assert(state.size() == 2);
        assert(dynamicsDerivative.rows() == 2 && dynamicsDerivative.cols() == 1);
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
        assert(state.size() == 2);
        assert(control.size() == 1);
        costValue = state(0) * state(0) + state(1) * state(1) + control(0) * control(0);
        return true;
     }

    virtual bool costFirstPartialDerivativeWRTState(double time,
                                                    const iDynTree::VectorDynSize& state,
                                                    const iDynTree::VectorDynSize& control,
                                                    iDynTree::VectorDynSize& partialDerivative) override {
        assert(state.size() == 2);
        assert(control.size() == 1);
        assert(partialDerivative.size() == 2);
        partialDerivative(0) = 2*state(0);
        partialDerivative(1) = 2*state(1);
        return true;
    }

    virtual bool costFirstPartialDerivativeWRTControl(double time,
                                                      const iDynTree::VectorDynSize& state,
                                                      const iDynTree::VectorDynSize& control,
                                                      iDynTree::VectorDynSize& partialDerivative) override {
        assert(state.size() == 2);
        assert(control.size() == 1);
        assert(partialDerivative.size() == 1);
        partialDerivative(0) = 2*control(0);
        return true;
    }

    virtual bool costSecondPartialDerivativeWRTState(double time,
                                                     const iDynTree::VectorDynSize& state,
                                                     const iDynTree::VectorDynSize& control,
                                                     iDynTree::MatrixDynSize& partialDerivative) override {
        assert(state.size() == 2);
        assert(control.size() == 1);
        assert(partialDerivative.rows() == 2 && partialDerivative.cols() == 2);
        partialDerivative.zero();
        partialDerivative(0,0) = 2;
        partialDerivative(1,1) = 2;
        return true;
    }

    virtual bool costSecondPartialDerivativeWRTControl(double time,
                                                       const iDynTree::VectorDynSize& state,
                                                       const iDynTree::VectorDynSize& control,
                                                       iDynTree::MatrixDynSize& partialDerivative) override {
        assert(state.size() == 2);
        assert(control.size() == 1);
        assert(partialDerivative.rows() == 1 && partialDerivative.cols() == 1);
        partialDerivative(0,0) = 2;
        return true;
    }


    virtual bool costSecondPartialDerivativeWRTStateControl(double time,
                                                            const iDynTree::VectorDynSize& state,
                                                            const iDynTree::VectorDynSize& control,
                                                            iDynTree::MatrixDynSize& partialDerivative) override {
        assert(state.size() == 2);
        assert(control.size() == 1);
        assert(partialDerivative.rows() == 2 && partialDerivative.cols() == 1);
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
        assert(setUpperBound(upperBound));
    }
    TestConstraint(const std::string& name)
        :iDynTree::optimalcontrol::Constraint(1, name)
    {
        iDynTree::VectorDynSize upperBound(1);
        upperBound(0) = 0.5;
        assert(setUpperBound(upperBound));
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

        assert(constraint.size() == 1);

        constraint(0) = control(0);

        return true;
    }

    virtual bool constraintJacobianWRTState(double time,
                                            const iDynTree::VectorDynSize& state,
                                            const iDynTree::VectorDynSize& control,
                                            iDynTree::MatrixDynSize& jacobian) override {
        assert((jacobian.rows() == 1) && (jacobian.cols() == 2));
        jacobian.zero();
        return true;
    }

    virtual bool constraintJacobianWRTControl(double time,
                                              const iDynTree::VectorDynSize& state,
                                              const iDynTree::VectorDynSize& control,
                                              iDynTree::MatrixDynSize& jacobian) override {
        assert((jacobian.rows() == 1) && (jacobian.cols() == 1));
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
    iDynTree::assertTrue(problem->setTimeHorizon(0.0, 10.0));
    iDynTree::assertTrue(problem->setDynamicalSystemConstraint(doubleIntegrator));
    iDynTree::assertTrue(problem->addLagrangeTerm(1.0, cost));
    bound(0) = 0.8;
    iDynTree::assertTrue(problem->setControlUpperBound(bound));
    bound(0) = -0.9;
    iDynTree::assertTrue(problem->setControlLowerBound(bound));

    iDynTree::assertTrue(group->addConstraint(constraint, iDynTree::optimalcontrol::TimeRange(6.0, 10.0)));
    iDynTree::assertTrue(problem->addGroupOfConstraints(group));

    // Multiple Shooting settings
    iDynTree::assertTrue(solver.setIntegrator(integrator));
    iDynTree::assertTrue(solver.setOptimizer(optimizer));
    iDynTree::assertTrue(solver.setStepSizeBounds(0.001, 0.01));
    iDynTree::assertTrue(solver.setControlPeriod(0.01));

    iDynTree::VectorDynSize initialState(2);
    iDynTree::getRandomVector(initialState, -2.0, 2.0);
    iDynTree::assertTrue(solver.setInitialState(initialState));


    // Optimizer settings
    //iDynTree::assertTrue(optimizer->setIpoptOption("linear_solver", "ma27"));
    iDynTree::assertTrue(optimizer->setIpoptOption("print_level", 0));
    iDynTree::assertTrue(optimizer->setIpoptOption("hessian_constant", "yes"));
    iDynTree::assertTrue(optimizer->setIpoptOption("jac_c_constant", "yes"));
    iDynTree::assertTrue(optimizer->setIpoptOption("jac_d_constant", "yes"));

    clock_t initT, endT;
    initT = clock();
    iDynTree::assertTrue(solver.solve());
    endT = clock();

    std::vector<iDynTree::VectorDynSize> states, controls;
    iDynTree::assertTrue(solver.getSolution(states, controls));
    std::cerr << "Initial state: " << initialState.toString() << std::endl;
    std::cerr << "First state: " << states.front().toString() << std::endl;
    std::cerr << "Last state: " << states.back().toString() << std::endl;
    std::cerr << "First control: " << controls.front().toString() << std::endl;
    std::cerr << "Last control: " << controls.back().toString() << std::endl;
    std::cerr << "Elapsed time: " <<  static_cast<double>(endT - initT) / CLOCKS_PER_SEC * 1000.0 << std::endl;

    for (int i=0; i < 10; ++i){

        std::cerr << "------------" << std::endl;

        //iDynTree::getRandomVector(initialState, -2.0, 2.0);
        initialState = states.front();
        iDynTree::assertTrue(solver.setInitialState(initialState));

        initT = clock();
        iDynTree::assertTrue(solver.solve());
        endT = clock();


        iDynTree::assertTrue(solver.getSolution(states, controls));
        std::cerr << "Initial state: " << initialState.toString() << std::endl;
        std::cerr << "First state: " << states.front().toString() << std::endl;
        std::cerr << "Last state: " << states.back().toString() << std::endl;
        std::cerr << "First control: " << controls.front().toString() << std::endl;
        std::cerr << "Last control: " << controls.back().toString() << std::endl;
        std::cerr << "Elapsed time: " <<  static_cast<double>(endT - initT) / CLOCKS_PER_SEC * 1000.0 << std::endl;
    }

    return EXIT_SUCCESS;
}
