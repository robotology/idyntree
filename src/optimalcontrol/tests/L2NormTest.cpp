// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause


#include <iDynTree/L2NormCost.h>
#include <iDynTree/TimeVaryingObject.h>
#include <iDynTree/TestUtils.h>
#include <iDynTree/EigenHelpers.h>
#include <memory>

int main() {

    iDynTree::VectorDynSize state(5), desiredStateValue(2), control(10);
    iDynTree::IndexRange stateRange, controlRange;

    iDynTree::getRandomVector(state);
    iDynTree::getRandomVector(desiredStateValue);
    iDynTree::getRandomVector(control);

    stateRange.offset = 1;
    stateRange.size = 2;

    controlRange.offset = 3;
    controlRange.size = 4;

    std::shared_ptr<iDynTree::optimalcontrol::L2NormCost> stateCost = std::make_shared<iDynTree::optimalcontrol::L2NormCost>("stateCost", stateRange, 5, iDynTree::IndexRange::InvalidRange(), 1);
    std::shared_ptr<iDynTree::optimalcontrol::Cost> controlCost = std::make_shared<iDynTree::optimalcontrol::L2NormCost>("controlCost", iDynTree::IndexRange::InvalidRange(), 1, controlRange, 10);
    std::shared_ptr<iDynTree::optimalcontrol::TimeVaryingVector> desiredState = std::make_shared<iDynTree::optimalcontrol::TimeInvariantVector>(desiredStateValue);


    iDynTree::MatrixDynSize stateWeight(2,2);
    stateWeight(0,0) = 2;
    stateWeight(1,1) = 2;

    double controlCostCHeck = 0.5 * iDynTree::toEigen(control).segment(controlRange.offset, controlRange.size).squaredNorm();
    double controlCostOutput;

    bool ok = controlCost ->costEvaluation(0.0, state, control, controlCostOutput);

    ASSERT_IS_TRUE(ok);
    ASSERT_EQUAL_DOUBLE(controlCostCHeck, controlCostOutput);

    ok = stateCost->setStateWeight(stateWeight);
    ASSERT_IS_TRUE(ok);

    ok = stateCost->setStateDesiredTrajectory(desiredState);
    ASSERT_IS_TRUE(ok);

    double stateCostCheck = (iDynTree::toEigen(state).segment(stateRange.offset, stateRange.size) - iDynTree::toEigen(desiredStateValue)).squaredNorm();
    double stateCostOutput;

    ok = stateCost->costEvaluation(0.0, state, control, stateCostOutput);

    ASSERT_IS_TRUE(ok);
    ASSERT_EQUAL_DOUBLE(stateCostCheck, stateCostOutput);

    iDynTree::VectorDynSize dummyVector;
    iDynTree::MatrixDynSize dummyMatrix;

    ok = stateCost->costFirstPartialDerivativeWRTState(0.0, state, control, dummyVector);
    ASSERT_IS_TRUE(ok);
    ok = stateCost->costFirstPartialDerivativeWRTControl(0.0, state, control, dummyVector);
    ASSERT_IS_TRUE(ok);
    ok = stateCost->costSecondPartialDerivativeWRTState(0.0, state, control, dummyMatrix);
    ASSERT_IS_TRUE(ok);
    ok = stateCost->costSecondPartialDerivativeWRTControl(0.0, state, control, dummyMatrix);
    ASSERT_IS_TRUE(ok);


    return EXIT_SUCCESS;

}
