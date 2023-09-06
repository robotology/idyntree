// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause
/*
 * Originally developed for Prioritized Optimal Control (2014)
 * Refactored in 2018.
 * Design inspired by
 * - ACADO toolbox (http://acado.github.io)
 * - ADRL Control Toolbox (https://adrlab.bitbucket.io/ct/ct_doc/doc/html/index.html)
 */

#include <iDynTree/OptimizationProblem.h>
#include <iDynTree/Optimizers/WorhpInterface.h>
#include <iDynTree/TestUtils.h>
#include <iDynTree/Utils.h>
#include <iDynTree/VectorDynSize.h>
#include <iDynTree/MatrixDynSize.h>
#include <vector>
#include <memory>

class TestProblem : public iDynTree::optimization::OptimizationProblem {
    double m_a1, m_b1, m_c1;
    double m_a2, m_b2, m_c2;
    double m_plusInfinity, m_minusInfinity;
    iDynTree::VectorDynSize m_variables, m_expectedVariables;

public:
    TestProblem()
    :m_plusInfinity(1e19)
    ,m_minusInfinity(-1e19)
    {}

    virtual ~TestProblem() override {}

    virtual bool prepare() override {
        m_a1 = iDynTree::getRandomDouble(0.01, 10.0);
        m_b1 = iDynTree::getRandomDouble(-10.0, 10.0);
        m_c1 = iDynTree::getRandomDouble(-10.0, 10.0);
        m_a2 = iDynTree::getRandomDouble(0.01, 10.0);
        m_b2 = iDynTree::getRandomDouble(-10.0, 10.0);
        m_c2 = iDynTree::getRandomDouble(-10.0, 10.0);

        return true;
    }

    virtual void reset() override {}

    virtual unsigned int numberOfVariables() override {
        return 2;
    }

    virtual unsigned int numberOfConstraints() override {
        return 2;
    }

    virtual bool getGuess(iDynTree::VectorDynSize &guess) override {
        guess.resize(2);
        guess.zero();
        return true;
    }

    virtual bool getConstraintsBounds(iDynTree::VectorDynSize& constraintsLowerBounds, iDynTree::VectorDynSize& constraintsUpperBounds) override {
        constraintsLowerBounds.resize(2);
        constraintsUpperBounds.resize(2);
        constraintsLowerBounds(0) = m_c1;
        constraintsLowerBounds(1) = m_c2;
        constraintsUpperBounds(0) = m_plusInfinity;
        constraintsUpperBounds(1) = m_plusInfinity;
        return true;
    }

    virtual bool getVariablesUpperBound(iDynTree::VectorDynSize& variablesUpperBound) override {
        return false;
    } //return false if not upper bounded

    virtual bool getVariablesLowerBound(iDynTree::VectorDynSize& variablesLowerBound) override {
        return false;
    } //return false if not lower bounded

    virtual bool getConstraintsJacobianInfo(std::vector<size_t>& nonZeroElementRows, std::vector<size_t>& nonZeroElementColumns) override {
        nonZeroElementRows.resize(2);
        nonZeroElementColumns.resize(2);

        nonZeroElementRows[0] = 0;
        nonZeroElementRows[1] = 1;

        nonZeroElementColumns = nonZeroElementRows;

        return true;
    }

    virtual bool getHessianInfo(std::vector<size_t>& nonZeroElementRows, std::vector<size_t>& nonZeroElementColumns) override {
        nonZeroElementRows.resize(2);
        nonZeroElementColumns.resize(2);

        nonZeroElementRows[0] = 0;
        nonZeroElementRows[1] = 1;

        nonZeroElementColumns = nonZeroElementRows;

        return true;
    } //costs and constraints together

    virtual bool setVariables(const iDynTree::VectorDynSize& variables) override {
        ASSERT_IS_TRUE(variables.size()==2);
        m_variables = variables;
        return true;
    }

    virtual bool evaluateCostFunction(double& costValue) override {
        costValue = m_a1 * m_variables(0) * m_variables(0) + m_b1 * m_variables(0) + m_a2 * m_variables(1) * m_variables(1) + m_b2 * m_variables(1);
        return true;
    }

    virtual bool evaluateCostGradient(iDynTree::VectorDynSize& gradient) override {
        gradient.resize(numberOfVariables());
        gradient(0) = 2 * m_a1 * m_variables(0) + m_b1;
        gradient(1) = 2 * m_a2 * m_variables(1) + m_b2;
        return true;
    }

    virtual bool evaluateCostHessian(iDynTree::MatrixDynSize& hessian) override {
        hessian.resize(2,2);
        hessian.zero();
        hessian(0,0) = 2*m_a1;
        hessian(1,1) = 2*m_a2;
        return true;
    }

    virtual bool evaluateConstraints(iDynTree::VectorDynSize& constraints) override {
        constraints.resize(2);
        constraints = m_variables;
        return true;
    }

    virtual bool evaluateConstraintsJacobian(iDynTree::MatrixDynSize& jacobian) override {
        jacobian.resize(2,2);
        jacobian(0,0) = 1.0;
        jacobian(1,1) = 1.0;
        return true;
    } //using dense matrices, but the sparsity pattern is still obtained

    virtual bool evaluateConstraintsHessian(const iDynTree::VectorDynSize& constraintsMultipliers, iDynTree::MatrixDynSize& hessian) override {
        hessian.resize(2,2);
        hessian.zero();
        return true;
    } //using dense matrices, but the sparsity pattern is still obtained

    bool setPlusInfinity(double plusInfinity) {
        if (plusInfinity < 0)
            return false;

        m_plusInfinity = plusInfinity;
        return true;
    }

    bool setMinusInfinity(double minusInfinity) {
        if (minusInfinity > 0)
            return false;

        m_minusInfinity = minusInfinity;
        return true;
    }

    double expectedMinimum() {
        double expected;
        if (m_c1 < (-m_b1/(2*m_a1)))
            expected = - m_b1 * m_b1 / (4*m_a1);
        else
            expected = m_a1*m_c1*m_c1 + m_b1*m_c1;

        if (m_c2 < (-m_b2/(2*m_a2)))
            expected += - m_b2 * m_b2 / (4*m_a2);
        else
            expected += m_a2*m_c2*m_c2 + m_b2*m_c2;

        return expected;
    }

    const iDynTree::VectorDynSize& expectedVariables() {
        m_expectedVariables.resize(2);
        if (m_c1 < (-m_b1/(2*m_a1)))
            m_expectedVariables(0) = - m_b1 / (2*m_a1);
        else
            m_expectedVariables(0) = m_c1;

        if (m_c2 < (-m_b2/(2*m_a2)))
            m_expectedVariables(1) = - m_b2 / (2*m_a2);
        else
            m_expectedVariables(1) = m_c2;

        return m_expectedVariables;
    }
};

int main(){
    iDynTree::optimization::WorhpInterface worhpSolver;
    std::shared_ptr<TestProblem> problem(new TestProblem);
    iDynTree::VectorDynSize guess(2), dummy1, dummy2, dummy3;
    guess.zero();

    ASSERT_IS_TRUE(worhpSolver.setWorhpParam("AcceptTolOpti", 1e-6));
    ASSERT_IS_TRUE(worhpSolver.setWorhpParam("AcceptTolFeas", 1e-6));
    ASSERT_IS_TRUE(worhpSolver.setWorhpParam("Algorithm", 2));
    ASSERT_IS_TRUE(worhpSolver.setWorhpParam("NLPprint", -1));


    ASSERT_IS_TRUE(problem->setMinusInfinity(worhpSolver.minusInfinity()));
    ASSERT_IS_TRUE(problem->setPlusInfinity(worhpSolver.plusInfinity()));
    ASSERT_IS_TRUE(worhpSolver.setProblem(problem));
    for (int i = 0; i < 5; ++i){
        ASSERT_IS_TRUE(worhpSolver.solve());
        double optimalCost;
        ASSERT_IS_TRUE(worhpSolver.getOptimalCost(optimalCost));
        ASSERT_EQUAL_DOUBLE_TOL(optimalCost, problem->expectedMinimum(), 1e-5);
        iDynTree::VectorDynSize solution;
        ASSERT_IS_TRUE(worhpSolver.getPrimalVariables(solution));
        ASSERT_EQUAL_VECTOR_TOL(solution, problem->expectedVariables(), 1e-5);
        ASSERT_IS_TRUE(worhpSolver.getDualVariables(dummy1, dummy2, dummy3));
    }
    return EXIT_SUCCESS;
}
