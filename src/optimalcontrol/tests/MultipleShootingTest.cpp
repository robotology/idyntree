// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include <iDynTree/OptimalControlProblem.h>
#include <iDynTree/ConstraintsGroup.h>
#include <iDynTree/DynamicalSystem.h>
#include <iDynTree/Constraint.h>
#include <iDynTree/Cost.h>
#include <iDynTree/Optimizer.h>
#include <iDynTree/OCSolvers/MultipleShootingSolver.h>
#include <iDynTree/DynamicalSystem.h>
#include <iDynTree/TestUtils.h>
#include <iDynTree/Utils.h>
#include <iDynTree/Integrators/ForwardEuler.h>
#include <iDynTree/VectorDynSize.h>
#include <iDynTree/MatrixDynSize.h>
#include <iDynTree/TimeRange.h>
#include <Eigen/Dense>
#include <iDynTree/EigenHelpers.h>
#include <string>

class TestSystem : public iDynTree::optimalcontrol::DynamicalSystem {
public:
    TestSystem() : iDynTree::optimalcontrol::DynamicalSystem(2,3) {}
    ~TestSystem() override;

    virtual bool dynamics(const iDynTree::VectorDynSize &state, double /*time*/, iDynTree::VectorDynSize &stateDynamics) override {
        if (state.size() != 2)
            return false;

        ASSERT_IS_TRUE(stateDynamics.size() == 2);

        stateDynamics(0) = state(0) + controlInput()(0) + 2 * controlInput()(1);
        stateDynamics(1) = state(1) + controlInput()(0) + 2 * controlInput()(2);
        return true;
    }

    virtual bool dynamicsStateFirstDerivative(const iDynTree::VectorDynSize& state,
                                              double /*time*/,
                                              iDynTree::MatrixDynSize& dynamicsDerivative) override {
        if (state.size() != 2)
            return false;

        ASSERT_IS_TRUE((dynamicsDerivative.rows() == 2) && (dynamicsDerivative.cols() == 2));

        dynamicsDerivative.zero();
        dynamicsDerivative(0, 0) = 1.0;
        dynamicsDerivative(1, 1) = 1.0;
        return true;
    }

    virtual bool dynamicsControlFirstDerivative(const iDynTree::VectorDynSize& state,
                                                double /*time*/,
                                                iDynTree::MatrixDynSize& dynamicsDerivative) override {
        if (state.size() != 2)
            return false;

        ASSERT_IS_TRUE((dynamicsDerivative.rows() == 2) && (dynamicsDerivative.cols() == 3));

        dynamicsDerivative.zero();
        dynamicsDerivative(0, 0) = 1.0;
        dynamicsDerivative(0, 1) = 2.0;
        dynamicsDerivative(1, 0) = 1.0;
        dynamicsDerivative(1, 2) = 2.0;
        return true;
    }

    virtual bool dynamicsStateFirstDerivativeSparsity(iDynTree::optimalcontrol::SparsityStructure& stateSparsity) override {
        iDynTree::optimalcontrol::SparsityStructure sparsity;
        sparsity.addIdentityBlock(0ul, 0, 2);
        stateSparsity = sparsity;
        return true;
    }

    virtual bool dynamicsControlFirstDerivativeSparsity(iDynTree::optimalcontrol::SparsityStructure& controlSparsity) override {
        iDynTree::optimalcontrol::SparsityStructure sparsity;
        sparsity.add(0, 0);
        sparsity.add(0, 1);
        sparsity.add(1, 0);
        sparsity.add(1, 2);
        controlSparsity = sparsity;
        return true;
    }

    virtual bool dynamicsSecondPartialDerivativeWRTState(double /*time*/,
                                                         const iDynTree::VectorDynSize& state,
                                                         const iDynTree::VectorDynSize& /*lambda*/,
                                                         iDynTree::MatrixDynSize& partialDerivative) override {
        partialDerivative.resize(state.size(), state.size());
        partialDerivative.zero();
        return true;
    }

    virtual bool dynamicsSecondPartialDerivativeWRTControl(double /*time*/,
                                                           const iDynTree::VectorDynSize& /*state*/,
                                                           const iDynTree::VectorDynSize& /*lambda*/,
                                                           iDynTree::MatrixDynSize& partialDerivative) override {
        partialDerivative.resize(controlInput().size(), controlInput().size());
        partialDerivative.zero();
        return true;
    }

    virtual bool dynamicsSecondPartialDerivativeWRTStateControl(double /*time*/,
                                                                const iDynTree::VectorDynSize& state,
                                                                const iDynTree::VectorDynSize& /*lambda*/,
                                                                iDynTree::MatrixDynSize& partialDerivative) override {
        partialDerivative.resize(state.size(), controlInput().size());
        partialDerivative.zero();
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
        ASSERT_IS_TRUE(setUpperBound(upperBound));
    }
    TestConstraint(const std::string& name)
        :iDynTree::optimalcontrol::Constraint(1, name)
    {
        iDynTree::VectorDynSize upperBound(1);
        upperBound(0) = 10;
        ASSERT_IS_TRUE(setUpperBound(upperBound));
    }
    virtual ~TestConstraint() override;

    virtual bool evaluateConstraint(double /*time*/,
                                    const iDynTree::VectorDynSize& state,
                                    const iDynTree::VectorDynSize& control,
                                    iDynTree::VectorDynSize& constraint) override {
        if (state.size() != 2)
            return false;

        if (control.size() != 3)
            return false;

        ASSERT_IS_TRUE(constraint.size() == 1);

        constraint(0) = control(0);

        return true;
    }

    virtual bool constraintJacobianWRTState(double /*time*/,
                                            const iDynTree::VectorDynSize& /*state*/,
                                            const iDynTree::VectorDynSize& /*control*/,
                                            iDynTree::MatrixDynSize& jacobian) override {
        ASSERT_IS_TRUE((jacobian.rows() == 1) && (jacobian.cols() == 2));
        jacobian.zero();
        return true;
    }

    virtual bool constraintJacobianWRTControl(double /*time*/,
                                              const iDynTree::VectorDynSize& /*state*/,
                                              const iDynTree::VectorDynSize& /*control*/,
                                              iDynTree::MatrixDynSize& jacobian) override {
        ASSERT_IS_TRUE((jacobian.rows() == 1) && (jacobian.cols() == 3));
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

    virtual bool constraintJacobianWRTStateSparsity(iDynTree::optimalcontrol::SparsityStructure& stateSparsity) override {
        stateSparsity.clear();
        return true;
    }

    virtual bool constraintJacobianWRTControlSparsity(iDynTree::optimalcontrol::SparsityStructure& controlSparsity) override {
        iDynTree::optimalcontrol::SparsityStructure sparsity;
        sparsity.add(0,0);
        controlSparsity = sparsity;
        return true;
    }

    virtual bool constraintSecondPartialDerivativeWRTState(double /*time*/,
                                                           const iDynTree::VectorDynSize& state,
                                                           const iDynTree::VectorDynSize& /*control*/,
                                                           const iDynTree::VectorDynSize& /*lambda*/,
                                                           iDynTree::MatrixDynSize& hessian) override {
        hessian.resize(state.size(), state.size());
        hessian.zero();
        return true;
    }

    virtual bool constraintSecondPartialDerivativeWRTControl(double /*time*/,
                                                             const iDynTree::VectorDynSize& /*state*/,
                                                             const iDynTree::VectorDynSize& control,
                                                             const iDynTree::VectorDynSize& /*lambda*/,
                                                             iDynTree::MatrixDynSize& hessian) override {
        hessian.resize(control.size(), control.size());
        hessian.zero();
        return true;
    }

    virtual bool constraintSecondPartialDerivativeWRTStateControl(double /*time*/,
                                                                  const iDynTree::VectorDynSize& state,
                                                                  const iDynTree::VectorDynSize& control,
                                                                  const iDynTree::VectorDynSize& /*lambda*/,
                                                                  iDynTree::MatrixDynSize& hessian) override {
        hessian.resize(state.size(), control.size());
        hessian.zero();
        return true;
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

    virtual ~TestCost() override;

    virtual bool costEvaluation(double /*time*/,
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

    virtual bool costFirstPartialDerivativeWRTState(double /*time*/,
                                                    const iDynTree::VectorDynSize& state,
                                                    const iDynTree::VectorDynSize& control,
                                                    iDynTree::VectorDynSize& partialDerivative) override {
        if (state.size() != 2)
            return false;
        if (control.size() != 3)
            return false;

        ASSERT_IS_TRUE(partialDerivative.size() == 2);

        partialDerivative(0) = 20 * state(0);
        partialDerivative(1) = 20 * state(1);

        return true;
    }

    virtual bool costFirstPartialDerivativeWRTControl(double /*time*/,
                                                      const iDynTree::VectorDynSize& state,
                                                      const iDynTree::VectorDynSize& control,
                                                      iDynTree::VectorDynSize& partialDerivative) override {
        if (state.size() != 2)
            return false;
        if (control.size() != 3)
            return false;

        ASSERT_IS_TRUE(partialDerivative.size() == 3);

        partialDerivative(0) = 2 * control(0);
        partialDerivative(1) = 2 * control(1);
        partialDerivative(2) = 2 * control(2);

        return true;
    }

    virtual bool costSecondPartialDerivativeWRTState(double /*time*/,
                                                     const iDynTree::VectorDynSize& state,
                                                     const iDynTree::VectorDynSize& control,
                                                     iDynTree::MatrixDynSize& partialDerivative) override {
        if (state.size() != 2)
            return false;
        if (control.size() != 3)
            return false;

        ASSERT_IS_TRUE((partialDerivative.rows() == 2) && (partialDerivative.cols() == 2));

        partialDerivative.zero();
        partialDerivative(0,0) = 20;
        partialDerivative(1,1) = 20;
        return true;
    }

    virtual bool costSecondPartialDerivativeWRTControl(double /*time*/,
                                                       const iDynTree::VectorDynSize& state,
                                                       const iDynTree::VectorDynSize& control,
                                                       iDynTree::MatrixDynSize& partialDerivative) override {
          if (state.size() != 2)
              return false;
          if (control.size() != 3)
              return false;

          ASSERT_IS_TRUE((partialDerivative.rows() == 3) && (partialDerivative.cols() == 3));

          partialDerivative.zero();
          partialDerivative(0, 0) = 2;
          partialDerivative(1, 1) = 2;
          partialDerivative(2, 2) = 2;
          return true;
    }

    virtual bool costSecondPartialDerivativeWRTStateControl(double /*time*/,
                                                            const iDynTree::VectorDynSize& state,
                                                            const iDynTree::VectorDynSize& control,
                                                            iDynTree::MatrixDynSize& partialDerivative) override {
        if (state.size() != 2)
            return false;
        if (control.size() != 3)
            return false;

        ASSERT_IS_TRUE((partialDerivative.rows() == 2) && (partialDerivative.cols() == 3));

        partialDerivative.zero();

        return true;
  }
};
TestCost::~TestCost(){}

class OptimizerTest : public iDynTree::optimization::Optimizer {

public:
    OptimizerTest() {}

    virtual ~OptimizerTest() override;

    virtual bool isAvailable() const override{
        return true;
    }

    virtual bool solve() override {
        iDynTree::VectorDynSize dummyVariables, dummy1, dummy2;
        iDynTree::MatrixDynSize dummyMatrix, jacobian;
        std::vector<size_t> dummy3, dummy4, nnzeroRows, nnzeroCols;
        double dummyCost;
        ASSERT_IS_TRUE(m_problem != nullptr);
        ASSERT_IS_TRUE(m_problem->prepare());
        dummyVariables.resize(m_problem->numberOfVariables());
        iDynTree::toEigen(dummyVariables).setConstant(1.0);
        ASSERT_IS_TRUE(m_problem->getConstraintsBounds(dummy1, dummy2));
        m_problem->getVariablesUpperBound(dummy1);
        m_problem->getVariablesLowerBound(dummy1);
        ASSERT_IS_TRUE(m_problem->getConstraintsJacobianInfo(nnzeroRows, nnzeroCols));
        ASSERT_IS_TRUE(nnzeroRows.size() == nnzeroCols.size());

        ASSERT_IS_TRUE(m_problem->getHessianInfo(dummy3, dummy4));
        ASSERT_IS_TRUE(m_problem->setVariables(dummyVariables));
        ASSERT_IS_TRUE(m_problem->evaluateCostFunction(dummyCost));
        ASSERT_IS_TRUE(m_problem->evaluateCostGradient(dummy1));
//        std::cerr << "Cost Gradient" << std::endl << dummy1.toString() << std::endl << std::endl;
        ASSERT_IS_TRUE(m_problem->evaluateCostHessian(dummyMatrix));
//        std::cerr << "Cost Hessian" << std::endl << dummyMatrix.toString() << std::endl << std::endl;
        ASSERT_IS_TRUE(m_problem->evaluateConstraints(dummy1));
        jacobian.resize(m_problem->numberOfConstraints(), m_problem->numberOfVariables());
        jacobian.zero();
        ASSERT_IS_TRUE(m_problem->evaluateConstraintsJacobian(jacobian));
        dummyMatrix.resize(jacobian.rows(), jacobian.cols());
        dummyMatrix.zero();

        for (size_t i =0; i < nnzeroRows.size(); ++i){
            jacobian(static_cast<unsigned int>(nnzeroRows[i]), static_cast<unsigned int>(nnzeroCols[i])) = 0;
        }

        ASSERT_EQUAL_MATRIX_TOL(dummyMatrix, jacobian, iDynTree::DEFAULT_TOL); //check the sparsity structure
//        std::cerr << "Cost Jacobian" << std::endl << dummyMatrix.toString() << std::endl << std::endl;
        iDynTree::MatrixDynSize dummyHessian;
        ASSERT_IS_TRUE(m_problem->evaluateConstraintsHessian(dummy1, dummyHessian));
        return true;
    }

    virtual bool getPrimalVariables(iDynTree::VectorDynSize &primalVariables) override {
        ASSERT_IS_TRUE(m_problem != nullptr);
        primalVariables.resize(m_problem->numberOfVariables());
        primalVariables.zero();
        return true;
    }

    virtual bool getDualVariables(iDynTree::VectorDynSize &constraintsMultipliers,
                                  iDynTree::VectorDynSize &lowerBoundsMultipliers,
                                  iDynTree::VectorDynSize &upperBoundsMultipliers) override {
        ASSERT_IS_TRUE(m_problem != nullptr);
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
        ASSERT_IS_TRUE(m_problem != nullptr);
        constraintsValues.resize(m_problem->numberOfConstraints());
        constraintsValues.zero();
        return true;
    }
};
OptimizerTest::~OptimizerTest() {}



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
    ASSERT_IS_TRUE(constraint2->setUpperBound(newBounds));
    double initTime = 1.0, endTime = 2.0;
    double minStep = 0.003, maxStep = 0.07, controlPeriod = 0.011;

    ASSERT_IS_TRUE(problem->setTimeHorizon(initTime, endTime));
    ASSERT_IS_TRUE(problem->dynamicalSystem().expired());
    ASSERT_IS_TRUE(problem->setDynamicalSystemConstraint(system));
    ASSERT_IS_TRUE(!(problem->dynamicalSystem().expired()));
    ASSERT_IS_TRUE(problem->addGroupOfConstraints(group1));
    ASSERT_IS_TRUE(group1->addConstraint(constraint2, iDynTree::optimalcontrol::TimeRange(4.0, 5.0)));
    ASSERT_IS_TRUE(problem->addConstraint(constraint1));
    ASSERT_IS_TRUE(problem->addLagrangeTerm(1.0, cost1));
    ASSERT_IS_TRUE(problem->addMayerTerm(1.0, cost2));

    ASSERT_IS_TRUE(solver.setIntegrator(integrator));
    ASSERT_IS_TRUE(solver.setStepSizeBounds(minStep, maxStep));
    ASSERT_IS_TRUE(solver.setControlPeriod(controlPeriod));
    ASSERT_IS_TRUE(solver.setOptimizer(optimizer));

    std::vector<double> stateTimings, controlTimings;
    ASSERT_IS_TRUE(solver.getPossibleTimings(stateTimings, controlTimings));
    for (size_t i = 0; i < stateTimings.size(); ++i){
        ASSERT_IS_TRUE((stateTimings[i] > initTime) && (stateTimings[i] <= endTime));
        if (i > 0)
            ASSERT_IS_TRUE(((stateTimings[i] - stateTimings[i-1]) >= (0.999 * minStep)) && ((stateTimings[i] - stateTimings[i-1]) <= (1.001 * maxStep)));
    }

    for (size_t i = 0; i < controlTimings.size(); ++i){
        ASSERT_IS_TRUE((controlTimings[i] >= initTime) && (controlTimings[i] <= endTime));
        if (i > 0)
            ASSERT_EQUAL_DOUBLE((controlTimings[i] - controlTimings[i-1]), controlPeriod);
    }

    ASSERT_IS_TRUE(solver.solve());

    minStep = 0.003;
    maxStep = 0.07;
    controlPeriod = 0.003;
    ASSERT_IS_TRUE(solver.setStepSizeBounds(minStep, maxStep));
    ASSERT_IS_TRUE(solver.setControlPeriod(controlPeriod));


    ASSERT_IS_TRUE(solver.getPossibleTimings(stateTimings, controlTimings));
    for (size_t i = 0; i < stateTimings.size(); ++i){
        ASSERT_IS_TRUE((stateTimings[i] > initTime) && (stateTimings[i] <= endTime));
        if (i > 0)
            ASSERT_IS_TRUE(((stateTimings[i] - stateTimings[i-1]) >= (0.999 * minStep)) && ((stateTimings[i] - stateTimings[i-1]) <= (1.001 * maxStep)));
    }

    for (size_t i = 0; i < controlTimings.size(); ++i){
        ASSERT_IS_TRUE((controlTimings[i] >= initTime) && (controlTimings[i] <= endTime));
        if (i > 0)
            ASSERT_EQUAL_DOUBLE((controlTimings[i] - controlTimings[i-1]), controlPeriod);
    }

    ASSERT_IS_TRUE(solver.solve());



    return EXIT_SUCCESS;
}
