#include <iDynTree/OptimalControlProblem.h>
#include <iDynTree/DynamicalSystem.h>
#include <iDynTree/Constraint.h>
#include <iDynTree/Cost.h>
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
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
    TestConstraint(std::string& name)
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

class TestCost : iDynTree::optimalcontrol::Cost {
public:
    TestCost()
    :iDynTree::optimalcontrol::Cost("testCost")
    {}

    TestCost(std::string& name)
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

int main() {
    return EXIT_SUCCESS;
}
