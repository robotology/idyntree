#include <iDynTree/OptimalControlProblem.h>
#include <iDynTree/ConstraintsGroup.h>
#include <iDynTree/DynamicalSystem.h>
#include <iDynTree/Constraint.h>
#include <iDynTree/Cost.h>
#include <iDynTree/Optimizer.h>
#include <iDynTree/OCSolvers/MultipleShootingSolver.h>
#include <iDynTree/DynamicalSystem.h>
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/Utils.h>
#include <iDynTree/Integrators/ForwardEuler.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/TimeRange.h>
#include <Eigen/Dense>
#include <iDynTree/Core/EigenHelpers.h>
#include <string>
#include <cassert>

class TestSystem : public iDynTree::optimalcontrol::DynamicalSystem {
public:
    TestSystem() : iDynTree::optimalcontrol::DynamicalSystem(2,3) {}
    ~TestSystem() override;

    virtual bool dynamics(const iDynTree::VectorDynSize &state, double time, iDynTree::VectorDynSize &stateDynamics) override {
        if (state.size() != 2)
            return false;

        assert(stateDynamics.size() == 2);

        stateDynamics(0) = state(0) + controlInput()(0) + 2 * controlInput()(1);
        stateDynamics(1) = state(1) + controlInput()(0) + 2 * controlInput()(2);
        return true;
    }

    virtual bool dynamicsStateFirstDerivative(const iDynTree::VectorDynSize& state,
                                              double time,
                                              iDynTree::MatrixDynSize& dynamicsDerivative) override {
        if (state.size() != 2)
            return false;

        assert((dynamicsDerivative.rows() == 2) && (dynamicsDerivative.cols() == 2));

        dynamicsDerivative.zero();
        dynamicsDerivative(0, 0) = 1.0;
        dynamicsDerivative(1, 1) = 1.0;
        return true;
    }

    virtual bool dynamicsControlFirstDerivative(const iDynTree::VectorDynSize& state,
                                                double time,
                                                iDynTree::MatrixDynSize& dynamicsDerivative) override {
        if (state.size() != 2)
            return false;

        assert((dynamicsDerivative.rows() == 2) && (dynamicsDerivative.cols() == 3));

        dynamicsDerivative.zero();
        dynamicsDerivative(0, 0) = 1.0;
        dynamicsDerivative(0, 1) = 2.0;
        dynamicsDerivative(1, 0) = 1.0;
        dynamicsDerivative(1, 2) = 2.0;
        return true;
    }
};
TestSystem::~TestSystem(){};

class TestConstraint : public iDynTree::optimalcontrol::Constraint {
public:
    TestConstraint()
        :iDynTree::optimalcontrol::Constraint(1, "testConstraint")
    {
        iDynTree::VectorDynSize upperBound(1);
        upperBound(0) = 10;
        assert(setUpperBound(upperBound));
    }
    TestConstraint(const std::string& name)
        :iDynTree::optimalcontrol::Constraint(1, name)
    {
        iDynTree::VectorDynSize upperBound(1);
        upperBound(0) = 10;
        assert(setUpperBound(upperBound));
    }
    virtual ~TestConstraint() override;

    virtual bool evaluateConstraint(double time,
                                    const iDynTree::VectorDynSize& state,
                                    const iDynTree::VectorDynSize& control,
                                    iDynTree::VectorDynSize& constraint) override {
        if (state.size() != 2)
            return false;

        if (control.size() != 3)
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
        assert((jacobian.rows() == 1) && (jacobian.cols() == 3));
        jacobian.zero();
        jacobian(0,0) = 1.0;

        return true;
    }

    virtual size_t expectedStateSpaceSize() const override {
        return 2;
    }

    virtual size_t expectedControlSpaceSize() const override {
        return 3;
    }
};
TestConstraint::~TestConstraint(){}

class TestCost : public iDynTree::optimalcontrol::Cost {
public:
    TestCost()
    :iDynTree::optimalcontrol::Cost("testCost")
    {}

    TestCost(const std::string& name)
        :iDynTree::optimalcontrol::Cost(name)
    {}

    virtual ~TestCost();

    virtual bool costEvaluation(double time,
                                const iDynTree::VectorDynSize& state,
                                const iDynTree::VectorDynSize& control,
                                double& costValue) override {
        if (state.size() != 2)
            return false;
        if (control.size() != 3)
            return false;

        costValue = 10 * state(0) * state(0) + 10 * state(1) * state(1) + control(0) * control(0) + control(1) * control(1) + control(2) * control(2);
        return true;
    }

    virtual bool costFirstPartialDerivativeWRTState(double time,
                                                    const iDynTree::VectorDynSize& state,
                                                    const iDynTree::VectorDynSize& control,
                                                    iDynTree::VectorDynSize& partialDerivative) override {
        if (state.size() != 2)
            return false;
        if (control.size() != 3)
            return false;

        assert(partialDerivative.size() == 2);

        partialDerivative(0) = 20 * state(0);
        partialDerivative(1) = 20 * state(1);

        return true;
    }

    virtual bool costFirstPartialDerivativeWRTControl(double time,
                                                      const iDynTree::VectorDynSize& state,
                                                      const iDynTree::VectorDynSize& control,
                                                      iDynTree::VectorDynSize& partialDerivative) override {
        if (state.size() != 2)
            return false;
        if (control.size() != 3)
            return false;

        assert(partialDerivative.size() == 3);

        partialDerivative(0) = 2 * control(0);
        partialDerivative(1) = 2 * control(1);
        partialDerivative(2) = 2 * control(2);

        return true;
    }

    virtual bool costSecondPartialDerivativeWRTState(double time,
                                                     const iDynTree::VectorDynSize& state,
                                                     const iDynTree::VectorDynSize& control,
                                                     iDynTree::MatrixDynSize& partialDerivative) override {
        if (state.size() != 2)
            return false;
        if (control.size() != 3)
            return false;

        assert((partialDerivative.rows() == 2) && (partialDerivative.cols() == 2));

        partialDerivative.zero();
        partialDerivative(0,0) = 20;
        partialDerivative(1,1) = 20;
        return true;
    }

    virtual bool costSecondPartialDerivativeWRTControl(double time,
                                                       const iDynTree::VectorDynSize& state,
                                                       const iDynTree::VectorDynSize& control,
                                                       iDynTree::MatrixDynSize& partialDerivative) override {
          if (state.size() != 2)
              return false;
          if (control.size() != 3)
              return false;

          assert((partialDerivative.rows() == 3) && (partialDerivative.cols() == 3));

          partialDerivative.zero();
          partialDerivative(0, 0) = 2;
          partialDerivative(1, 1) = 2;
          partialDerivative(2, 2) = 2;
          return true;
    }

    virtual bool costSecondPartialDerivativeWRTStateControl(double time,
                                                            const iDynTree::VectorDynSize& state,
                                                            const iDynTree::VectorDynSize& control,
                                                            iDynTree::MatrixDynSize& partialDerivative) override {
        if (state.size() != 2)
            return false;
        if (control.size() != 3)
            return false;

        assert((partialDerivative.rows() == 2) && (partialDerivative.cols() == 3));

        partialDerivative.zero();

        return true;
  }
};
TestCost::~TestCost(){}

class OptimizerTest : public iDynTree::optimization::Optimizer {

public:
    OptimizerTest() {}

    virtual ~OptimizerTest() override {}

    virtual bool setInitialGuess(iDynTree::VectorDynSize &initialGuess) override{
        return true;
    }

    virtual bool solve() override {
        iDynTree::VectorDynSize dummyVariables, dummy1, dummy2;
        iDynTree::MatrixDynSize dummyMatrix, jacobian;
        std::vector<size_t> dummy3, dummy4, nnzeroRows, nnzeroCols;
        double dummyCost;
        assert(m_problem);
        iDynTree::assertTrue(m_problem->prepare());
        dummyVariables.resize(m_problem->numberOfVariables());
        iDynTree::toEigen(dummyVariables).setConstant(1.0);
        iDynTree::assertTrue(m_problem->getConstraintsBounds(dummy1, dummy2));
        m_problem->getVariablesUpperBound(dummy1);
        m_problem->getVariablesLowerBound(dummy1);
        iDynTree::assertTrue(m_problem->getConstraintsJacobianInfo(nnzeroRows, nnzeroCols));
        assert(nnzeroRows.size() == nnzeroCols.size());

        iDynTree::assertTrue(m_problem->getHessianInfo(dummy3, dummy4));
        iDynTree::assertTrue(m_problem->setVariables(dummyVariables));
        iDynTree::assertTrue(m_problem->evaluateCostFunction(dummyCost));
        iDynTree::assertTrue(m_problem->evaluateCostGradient(dummy1));
//        std::cerr << "Cost Gradient" << std::endl << dummy1.toString() << std::endl << std::endl;
        iDynTree::assertTrue(m_problem->evaluateCostHessian(dummyMatrix));
//        std::cerr << "Cost Hessian" << std::endl << dummyMatrix.toString() << std::endl << std::endl;
        iDynTree::assertTrue(m_problem->evaluateConstraints(dummy1));
        jacobian.resize(m_problem->numberOfConstraints(), m_problem->numberOfVariables());
        jacobian.zero();
        iDynTree::assertTrue(m_problem->evaluateConstraintsJacobian(jacobian));
        dummyMatrix.resize(jacobian.rows(), jacobian.cols());
        dummyMatrix.zero();

        for (size_t i =0; i < nnzeroRows.size(); ++i){
            jacobian(nnzeroRows[i], nnzeroCols[i]) = 0;
        }

        iDynTree::assertMatrixAreEqual(dummyMatrix, jacobian, iDynTree::DEFAULT_TOL, "", 0); //check the sparsity structure
//        std::cerr << "Cost Jacobian" << std::endl << dummyMatrix.toString() << std::endl << std::endl;
        //not evaluating the constraint hessian for the moment

        return true;
    }

    virtual bool getPrimalVariables(iDynTree::VectorDynSize &primalVariables) override {
        assert(m_problem);
        primalVariables.resize(m_problem->numberOfVariables());
        primalVariables.zero();
        return true;
    }

    virtual bool getDualVariables(iDynTree::VectorDynSize &constraintsMultipliers,
                                  iDynTree::VectorDynSize &lowerBoundsMultipliers,
                                  iDynTree::VectorDynSize &upperBoundsMultipliers) override {
        assert(m_problem);
        constraintsMultipliers.resize(m_problem->numberOfConstraints());
        lowerBoundsMultipliers.resize(m_problem->numberOfVariables());
        upperBoundsMultipliers.resize(m_problem->numberOfVariables());
        return true;
    }

    virtual bool getOptimalCost(double &optimalCost) override {
        optimalCost = 0;
        return true;
    }

    virtual bool getOptimalConstraintsValues(iDynTree::VectorDynSize &constraintsValues) override {
        assert(m_problem);
        constraintsValues.resize(m_problem->numberOfConstraints());
        constraintsValues.zero();
        return true;
    }
};


int main(){

    //Definition
    std::shared_ptr<iDynTree::optimalcontrol::integrators::ForwardEuler> integrator(new iDynTree::optimalcontrol::integrators::ForwardEuler());
    std::shared_ptr<iDynTree::optimalcontrol::OptimalControlProblem> problem(new iDynTree::optimalcontrol::OptimalControlProblem());
    std::shared_ptr<OptimizerTest> optimizer(new OptimizerTest());
    iDynTree::optimalcontrol::MultipleShootingSolver solver(problem);
    std::shared_ptr<TestSystem> system(new TestSystem());
    std::shared_ptr<TestConstraint> constraint1(new TestConstraint("constraint1"));
    std::shared_ptr<TestConstraint> constraint2(new TestConstraint("constraint2"));
    std::shared_ptr<iDynTree::optimalcontrol::ConstraintsGroup> group1(new iDynTree::optimalcontrol::ConstraintsGroup("group1", 1));
    std::shared_ptr<TestCost> cost1(new TestCost("cost1"));
    std::shared_ptr<TestCost> cost2(new TestCost("cost2"));


    //Set-up
    iDynTree::VectorDynSize newBounds(1);
    newBounds(0) = 5.0;
    iDynTree::assertTrue(constraint2->setUpperBound(newBounds));
    double initTime = 1.0, endTime = 5.0;
    double minStep = 0.003, maxStep = 0.07, controlPeriod = 0.011;

    iDynTree::assertTrue(problem->setTimeHorizon(initTime, endTime));
    iDynTree::assertTrue(problem->dynamicalSystem().expired());
    iDynTree::assertTrue(problem->setDynamicalSystemConstraint(system));
    iDynTree::assertTrue(!(problem->dynamicalSystem().expired()));
    iDynTree::assertTrue(problem->addGroupOfConstraints(group1));
    iDynTree::assertTrue(group1->addConstraint(constraint2, iDynTree::optimalcontrol::TimeRange(4.0, 5.0)));
    iDynTree::assertTrue(problem->addContraint(constraint1));
    iDynTree::assertTrue(problem->addLagrangeTerm(1.0, cost1));
    iDynTree::assertTrue(problem->addMayerTerm(1.0, cost2));

    iDynTree::assertTrue(solver.setIntegrator(integrator));
    iDynTree::assertTrue(solver.setStepSizeBounds(minStep, maxStep));
    iDynTree::assertTrue(solver.setControlPeriod(controlPeriod));
    iDynTree::assertTrue(solver.setOptimizer(optimizer));

    std::vector<double> stateTimings, controlTimings;
    iDynTree::assertTrue(solver.getTimings(stateTimings, controlTimings));
    for (size_t i = 0; i < stateTimings.size(); ++i){
        iDynTree::assertTrue((stateTimings[i] > initTime) && (stateTimings[i] <= endTime));
        if (i > 0)
            iDynTree::assertTrue(((stateTimings[i] - stateTimings[i-1]) >= minStep) && ((stateTimings[i] - stateTimings[i-1]) <= maxStep));
    }

    for (size_t i = 0; i < controlTimings.size(); ++i){
        iDynTree::assertTrue((controlTimings[i] >= initTime) && (controlTimings[i] <= endTime));
        if (i > 0)
            iDynTree::assertDoubleAreEqual((controlTimings[i] - controlTimings[i-1]), controlPeriod);
    }

    iDynTree::assertTrue(solver.solve());


    return EXIT_SUCCESS;
}
