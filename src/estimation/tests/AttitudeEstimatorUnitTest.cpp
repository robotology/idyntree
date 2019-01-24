/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Estimation/AttitudeQuaternionEKF.h>
#include <iDynTree/Estimation/AttitudeMahonyFilter.h>
#include <iDynTree/Core/TestUtils.h>
#include <iostream>

void run(iDynTree::IAttitudeEstimator* estimator,
         const iDynTree::LinAcceleration& acc,
         const iDynTree::GyroscopeMeasurements& gyro,
         const iDynTree::MagnetometerMeasurements& mag)
{
    std::cout << "Propagating states..." << std::endl;
    estimator->propagateStates();
    std::cout << "Update measurements..." << std::endl;
    estimator->updateFilterWithMeasurements(acc, gyro, mag);
}

int main()
{
    std::unique_ptr<iDynTree::AttitudeQuaternionEKF> qEKF;

    qEKF = std::make_unique<iDynTree::AttitudeQuaternionEKF>();
    iDynTree::AttitudeQuaternionEKFParameters params;

    size_t x_size = qEKF->getInternalStateSize();

    // calling setParams resets and intializes the filter
    qEKF->setParameters(params);
    bool ok = qEKF->initializeFilter();
    ASSERT_IS_TRUE(ok);
    std::cout << "Propagate states will internally run EKF predict step" << std::endl;
    std::cout << "if setParams() was not called before, calling propagateStates before setting internal state will throw initial state not set error...." << std::endl;
    ok = qEKF->propagateStates();
    ASSERT_IS_TRUE(ok);
    std::cout << "Print.... OK" << std::endl;

    iDynTree::VectorDynSize x0;
    x0.resize(10);
    x0.zero();
    x0(0) = 0.5;
    x0(1) = -0.5;
    x0(2) = 0.5;
    x0(3) = -0.5;

    iDynTree::Span<double> x0_span(x0.data(), x0.size());
    std::cout << "Setting initial internal state" << std::endl;
    ok = qEKF->setInternalState(x0_span);
    ASSERT_IS_TRUE(ok);

    std::cout << "Calling use magnetometer method resets filter flags and re-initialized filter with current state as intial state" << std::endl;
    qEKF->useMagnetometerMeasurements(true);
    std::cout << "Since initial state is already set, calling propagate states will be succesful" << std::endl;
    ok = qEKF->propagateStates();
    ASSERT_IS_TRUE(ok);

    iDynTree::LinAcceleration linAcc; linAcc.zero();
    iDynTree::GyroscopeMeasurements gyro; gyro.zero();
    iDynTree::MagnetometerMeasurements mag; mag.zero();
    std::cout << "Update measurements will internally run EKF update step" << std::endl;
    ok = qEKF->updateFilterWithMeasurements(linAcc, gyro, mag);
    ASSERT_IS_TRUE(ok);

    ok = qEKF->useMagnetometerMeasurements(false);
    ASSERT_IS_TRUE(ok);
    ok = qEKF->updateFilterWithMeasurements(linAcc, gyro);
    ASSERT_IS_TRUE(ok);

    iDynTree::IAttitudeEstimator* qekf_(qEKF.get());
    for ( ; ; )
    {
        run(qekf_, linAcc, gyro, mag);
    }

    return EXIT_SUCCESS;
}
