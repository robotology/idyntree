#include <iDynTree/Optimizer.h>
#include <iDynTree/OCSolvers/MultipleShootingSolver.h>
#include <iDynTree/DynamicalSystem.h>
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/Utils.h>
#include <iDynTree/Integrators/ForwardEuler.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <cassert>

class OptimizerTest : public iDynTree::optimization::Optimizer {

public:
    OptimizerTest() {}

    virtual ~OptimizerTest() override {}

    virtual bool setInitialGuess(iDynTree::VectorDynSize &initialGuess) override{
        return true;
    }

    virtual bool solve() override {
        iDynTree::VectorDynSize dummyVariables, dummy1, dummy2;
        iDynTree::MatrixDynSize dummyMatrix;
        std::vector<size_t> dummy3, dummy4;
        double dummyCost;
        assert(m_problem);
        dummyVariables.resize(m_problem->numberOfVariables());
        dummyVariables.zero();
        iDynTree::assertTrue(m_problem->prepare());
        iDynTree::assertTrue(m_problem->getConstraintsBounds(dummy1, dummy2));
        m_problem->getVariablesUpperBound(dummy1);
        m_problem->getVariablesLowerBound(dummy1);
        iDynTree::assertTrue(m_problem->getConstraintsJacobianInfo(dummy3, dummy4));
        iDynTree::assertTrue(m_problem->getHessianInfo(dummy3, dummy4));
        iDynTree::assertTrue(m_problem->setVariables(dummyVariables));
        iDynTree::assertTrue(m_problem->evaluateCostFunction(dummyCost));
        iDynTree::assertTrue(m_problem->evaluateCostGradient(dummy1));
        iDynTree::assertTrue(m_problem->evaluateCostHessian(dummyMatrix));
        iDynTree::assertTrue(m_problem->evaluateConstraints(dummy1));
        iDynTree::assertTrue(m_problem->evaluateConstraintsJacobian(dummyMatrix));
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

    return EXIT_SUCCESS;
}
