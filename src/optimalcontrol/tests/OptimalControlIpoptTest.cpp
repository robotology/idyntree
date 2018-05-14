#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/DynamicalSystem.h>
#include <iDynTree/Cost.h>
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

int main () {
    std::shared_ptr<iDynTree::optimalcontrol::OptimalControlProblem> problem(new iDynTree::optimalcontrol::OptimalControlProblem());
    std::shared_ptr<DoubleIntegrator> doubleIntegrator(new DoubleIntegrator());
    std::shared_ptr<TestCost> cost(new TestCost);
    std::shared_ptr<iDynTree::optimalcontrol::ForwardEuler> integrator(new iDynTree::optimalcontrol::ForwardEuler);
    std::shared_ptr<iDynTree::optimization::IpoptInterface> optimizer(new iDynTree::optimization::IpoptInterface);
    iDynTree::optimalcontrol::MultipleShootingSolver solver(problem);

    iDynTree::assertTrue(problem->setTimeHorizon(0.0, 10.0));
    iDynTree::assertTrue(problem->setDynamicalSystemConstraint(doubleIntegrator));
    iDynTree::assertTrue(problem->addLagrangeTerm(1.0, cost));

    iDynTree::assertTrue(solver.setIntegrator(integrator));
    iDynTree::assertTrue(solver.setOptimizer(optimizer));
    iDynTree::assertTrue(solver.setStepSizeBounds(0.001, 0.01));
    iDynTree::assertTrue(solver.setControlPeriod(0.01));

    iDynTree::VectorDynSize initialState(2);
    iDynTree::getRandomVector(initialState, -2.0, 2.0);
    iDynTree::assertTrue(solver.setInitialState(initialState));

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
    std::cerr << "Firts control: " << controls.front().toString() << std::endl;
    std::cerr << "Last control: " << controls.back().toString() << std::endl;
    std::cerr << "Elapsed time: " <<  static_cast<double>(endT - initT) / CLOCKS_PER_SEC * 1000.0 << std::endl;

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
    std::cerr << "Firts control: " << controls.front().toString() << std::endl;
    std::cerr << "Last control: " << controls.back().toString() << std::endl;
    std::cerr << "Elapsed time: " <<  static_cast<double>(endT - initT) / CLOCKS_PER_SEC * 1000.0 << std::endl;




    return EXIT_SUCCESS;
}
