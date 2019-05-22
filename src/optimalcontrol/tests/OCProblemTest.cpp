/*
 * Copyright (C) 2014,2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 *
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include <iDynTree/OptimalControlProblem.h>
#include <iDynTree/DynamicalSystem.h>
#include <iDynTree/Constraint.h>
#include <iDynTree/Cost.h>
#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/ConstraintsGroup.h>
#include <iDynTree/TimeRange.h>
#include <Eigen/Dense>
#include <iDynTree/Core/EigenHelpers.h>
#include <string>

class TestSystem : public iDynTree::optimalcontrol::DynamicalSystem {
public:
    TestSystem() : iDynTree::optimalcontrol::DynamicalSystem(2,3) {}
    ~TestSystem() override;

    virtual bool dynamics(const iDynTree::VectorDynSize &state, double time, iDynTree::VectorDynSize &stateDynamics) override {
        if (state.size() != 2)
            return false;

        ASSERT_IS_TRUE(stateDynamics.size() == 2);

        stateDynamics(0) = state(0) + controlInput()(0) + 2 * controlInput()(1);
        stateDynamics(1) = state(1) + controlInput()(0) + 2 * controlInput()(2);
        return true;
    }

    virtual bool dynamicsStateFirstDerivative(const iDynTree::VectorDynSize& state,
                                              double time,
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
                                                double time,
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

    virtual bool evaluateConstraint(double time,
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

        ASSERT_IS_TRUE(partialDerivative.size() == 2);

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

        ASSERT_IS_TRUE(partialDerivative.size() == 3);

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

        ASSERT_IS_TRUE((partialDerivative.rows() == 2) && (partialDerivative.cols() == 2));

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

          ASSERT_IS_TRUE((partialDerivative.rows() == 3) && (partialDerivative.cols() == 3));

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

        ASSERT_IS_TRUE((partialDerivative.rows() == 2) && (partialDerivative.cols() == 3));

        partialDerivative.zero();

        return true;
  }
};
TestCost::~TestCost(){}

int main() {
    iDynTree::optimalcontrol::OptimalControlProblem problem;

    //Definition
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

    ASSERT_IS_TRUE(problem.setTimeHorizon(1.0, 5.0));
    ASSERT_IS_TRUE(problem.dynamicalSystem().expired());
    ASSERT_IS_TRUE(problem.setDynamicalSystemConstraint(system));
    ASSERT_IS_TRUE(!(problem.dynamicalSystem().expired()));
    ASSERT_IS_TRUE(problem.addGroupOfConstraints(group1));
    ASSERT_IS_TRUE(group1->addConstraint(constraint2, iDynTree::optimalcontrol::TimeRange(4.0, 5.0)));
    ASSERT_IS_TRUE(problem.addConstraint(constraint1));
    ASSERT_IS_TRUE(problem.addLagrangeTerm(1.0, cost1));
    ASSERT_IS_TRUE(problem.addMayerTerm(1.0, cost2));

    iDynTree::VectorDynSize testState(2), testControl(3);
    iDynTree::getRandomVector(testState, -10.0, 10.0);
    iDynTree::getRandomVector(testControl, -10.0, 10.0);

    //--------------- Checking Cost

    double expectedCost1, expectedCost2, obtainedCost;
    iDynTree::VectorDynSize expectedGradient1(2), expectedGradient2(2), gradientSum(2), obtainedGradient(2), expectedCtrlGradient1(3), expectedCtrlGradient2(3), ctrlSum(3), obtainedCtrlGradient(3);

    //Test before changing time range
    ASSERT_IS_TRUE(cost1->costEvaluation(0.0, testState, testControl, expectedCost1));
    ASSERT_IS_TRUE(cost1->costFirstPartialDerivativeWRTState(0.0, testState, testControl, expectedGradient1));
    ASSERT_IS_TRUE(cost1->costFirstPartialDerivativeWRTControl(0.0, testState, testControl, expectedCtrlGradient1));
    ASSERT_IS_TRUE(cost2->costEvaluation(0.0, testState, testControl, expectedCost2));
    ASSERT_IS_TRUE(cost2->costFirstPartialDerivativeWRTState(0.0, testState, testControl, expectedGradient2));
    ASSERT_IS_TRUE(cost2->costFirstPartialDerivativeWRTControl(0.0, testState, testControl, expectedCtrlGradient2));
    iDynTree::toEigen(gradientSum) = iDynTree::toEigen(expectedGradient1) + iDynTree::toEigen(expectedGradient2);
    iDynTree::toEigen(ctrlSum) = iDynTree::toEigen(expectedCtrlGradient1) + iDynTree::toEigen(expectedCtrlGradient2);

    ASSERT_IS_TRUE(problem.costsEvaluation(4.0, testState, testControl, obtainedCost));
    ASSERT_EQUAL_DOUBLE(expectedCost1, obtainedCost);

    ASSERT_IS_TRUE(problem.costsFirstPartialDerivativeWRTState(4.0, testState, testControl, obtainedGradient));
    ASSERT_EQUAL_VECTOR_TOL(expectedGradient1, obtainedGradient,iDynTree::DEFAULT_TOL);

    ASSERT_IS_TRUE(problem.costsFirstPartialDerivativeWRTControl(4.0, testState, testControl, obtainedCtrlGradient));
    ASSERT_EQUAL_VECTOR_TOL(expectedCtrlGradient1, obtainedCtrlGradient,iDynTree::DEFAULT_TOL);

    ASSERT_IS_TRUE(problem.costsEvaluation(5.0, testState, testControl, obtainedCost));
    ASSERT_EQUAL_DOUBLE(expectedCost1 + expectedCost2, obtainedCost);

    ASSERT_IS_TRUE(problem.costsFirstPartialDerivativeWRTState(5.0, testState, testControl, obtainedGradient));
    ASSERT_EQUAL_VECTOR_TOL(gradientSum, obtainedGradient,iDynTree::DEFAULT_TOL);

    ASSERT_IS_TRUE(problem.costsFirstPartialDerivativeWRTControl(5.0, testState, testControl, obtainedCtrlGradient));
    ASSERT_EQUAL_VECTOR_TOL(ctrlSum, obtainedCtrlGradient,iDynTree::DEFAULT_TOL);

    // Changing time horizon
    ASSERT_IS_TRUE(problem.setTimeHorizon(1.0, 5.5));

    //Test after changing time range
    ASSERT_IS_TRUE(problem.costsEvaluation(5.0, testState, testControl, obtainedCost));
    ASSERT_EQUAL_DOUBLE(expectedCost1, obtainedCost);

    ASSERT_IS_TRUE(problem.costsFirstPartialDerivativeWRTState(5.0, testState, testControl, obtainedGradient));
    ASSERT_EQUAL_VECTOR_TOL(expectedGradient1, obtainedGradient,iDynTree::DEFAULT_TOL);

    ASSERT_IS_TRUE(problem.costsFirstPartialDerivativeWRTControl(5.0, testState, testControl, obtainedCtrlGradient));
    ASSERT_EQUAL_VECTOR_TOL(expectedCtrlGradient1, obtainedCtrlGradient,iDynTree::DEFAULT_TOL);

    ASSERT_IS_TRUE(problem.costsEvaluation(5.5, testState, testControl, obtainedCost));
    ASSERT_EQUAL_DOUBLE(expectedCost1 + expectedCost2, obtainedCost);

    ASSERT_IS_TRUE(problem.costsFirstPartialDerivativeWRTState(5.5, testState, testControl, obtainedGradient));
    ASSERT_EQUAL_VECTOR_TOL(gradientSum, obtainedGradient,iDynTree::DEFAULT_TOL);

    ASSERT_IS_TRUE(problem.costsFirstPartialDerivativeWRTControl(5.5, testState, testControl, obtainedCtrlGradient));
    ASSERT_EQUAL_VECTOR_TOL(ctrlSum, obtainedCtrlGradient,iDynTree::DEFAULT_TOL);

    //---------Checking Constraints
    iDynTree::VectorDynSize expectedConstraints(2), obtainedConstraints;
    iDynTree::MatrixDynSize expectedStatejac(2,2), expectedControlJac(2,3), obtainedStatejac, obtainedControlJac;
    expectedStatejac.zero();
    expectedControlJac.zero();
    expectedControlJac(0,0) = 1.0;
    expectedControlJac(1,0) = 1.0;

    expectedConstraints(0) = testControl(0);
    expectedConstraints(1) = testControl(0);
    ASSERT_IS_TRUE(problem.constraintsEvaluation(4.0, testState, testControl, obtainedConstraints));
    ASSERT_EQUAL_VECTOR_TOL(expectedConstraints, obtainedConstraints, iDynTree::DEFAULT_TOL);
    ASSERT_IS_TRUE(problem.constraintsJacobianWRTState(4.0, testState, testControl, obtainedStatejac));
    ASSERT_EQUAL_MATRIX_TOL(expectedStatejac, obtainedStatejac, iDynTree::DEFAULT_TOL);
    ASSERT_IS_TRUE(problem.constraintsJacobianWRTControl(4.0, testState, testControl, obtainedControlJac));
    ASSERT_EQUAL_MATRIX_TOL(expectedControlJac, obtainedControlJac, iDynTree::DEFAULT_TOL);
    testControl(0) = 6.0;
    ASSERT_IS_TRUE(problem.isFeasiblePoint(3.0, testState, testControl));
    ASSERT_IS_FALSE(problem.isFeasiblePoint(4.0, testState, testControl));

    iDynTree::optimalcontrol::SparsityStructure stateSparsity, controlSparsity;
    ASSERT_IS_TRUE(problem.constraintsJacobianWRTStateSparsity(stateSparsity));
    ASSERT_IS_TRUE(problem.constraintsJacobianWRTControlSparsity(controlSparsity));
    iDynTree::MatrixDynSize stateSparsityCheck, controlSparsityCheck, stateZeroCheck, controlZeroCheck;
    stateSparsityCheck = obtainedStatejac;
    stateZeroCheck = stateSparsityCheck;
    stateZeroCheck.zero();
    controlSparsityCheck = obtainedControlJac;
    controlZeroCheck = controlSparsityCheck;
    controlZeroCheck.zero();

    for(size_t i = 0; i < stateSparsity.size(); ++i) {
        unsigned int row = static_cast<unsigned int>(stateSparsity[i].row);
        unsigned int col = static_cast<unsigned int>(stateSparsity[i].col);
        stateSparsityCheck(row, col) = 0.0;
    }

    ASSERT_EQUAL_MATRIX(stateSparsityCheck, stateZeroCheck);

    for(size_t i = 0; i < controlSparsity.size(); ++i) {
        unsigned int row = static_cast<unsigned int>(controlSparsity[i].row);
        unsigned int col = static_cast<unsigned int>(controlSparsity[i].col);
        controlSparsityCheck(row, col) = 0.0;
    }

    ASSERT_EQUAL_MATRIX(controlSparsityCheck, controlZeroCheck);


    return EXIT_SUCCESS;
}
