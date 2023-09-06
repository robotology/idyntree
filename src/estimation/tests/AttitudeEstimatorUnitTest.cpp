// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/AttitudeQuaternionEKF.h>
#include <iDynTree/AttitudeMahonyFilter.h>
#include <iDynTree/TestUtils.h>
#include <iDynTree/SpatialMotionVector.h>
#include <iDynTree/SpatialForceVector.h>
#include <iostream>
#include <memory>

void run(iDynTree::IAttitudeEstimator* estimator,
         const iDynTree::LinAcceleration& acc,
         const iDynTree::GyroscopeMeasurements& gyro,
         const iDynTree::MagnetometerMeasurements& mag)
{
    std::cout << "Propagating states..." << std::endl;
    estimator->propagateStates();
    std::cout << "Update measurements..." << std::endl;
    estimator->updateFilterWithMeasurements(acc, gyro, mag);
    iDynTree::RPY rpy;
    estimator->getOrientationEstimateAsRPY(rpy);
    std::cout << "Estimated orientation in RPY: " << rpy.toString() << std::endl;
}

int main()
{
    std::unique_ptr<iDynTree::AttitudeQuaternionEKF> qEKF;

    qEKF = std::make_unique<iDynTree::AttitudeQuaternionEKF>();
    iDynTree::AttitudeQuaternionEKFParameters params;

    params.time_step_in_seconds = 0.010;
    params.accelerometer_noise_variance = 0.03;
    params.magnetometer_noise_variance = 0.0;
    params.gyroscope_noise_variance = 0.5;
    params.gyro_bias_noise_variance = 10e-11;
    params.initial_orientation_error_variance = 10e-6;
    params.initial_ang_vel_error_variance = 10e-1;
    params.initial_gyro_bias_error_variance = 10e-11;
    params.bias_correlation_time_factor = 10e-3;
    params.use_magnetometer_measurements = false;

    size_t x_size = qEKF->getInternalStateSize();

    // calling setParams resets and intializes the filter
    qEKF->setParameters(params);
    bool ok = qEKF->initializeFilter();
    ASSERT_IS_TRUE(ok);
    std::cout << "Propagate states will internally run EKF predict step" << std::endl;
    std::cout << "calling propagateStates before setting internal state will throw initial state not set error...." << std::endl;
    ok = qEKF->propagateStates();
    ASSERT_IS_FALSE(ok);
    std::cout << "Print.... OK" << std::endl;

    iDynTree::VectorDynSize x0;
    x0.resize(10);
    x0.zero();
    x0(0) = 1.0;
    x0(1) = 0.0;
    x0(2) = 0.0;
    x0(3) = 0.0;

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
    ok = qEKF->updateFilterWithMeasurements(linAcc, gyro, mag);
    ASSERT_IS_FALSE(ok);
    linAcc(2) = -9.8;
    mag(2) = 1.0;
    std::cout << "Update measurements will internally run EKF update step" << std::endl;
    ok = qEKF->updateFilterWithMeasurements(linAcc, gyro, mag);
    ASSERT_IS_TRUE(ok);

    ok = qEKF->useMagnetometerMeasurements(false);
    ASSERT_IS_TRUE(ok);
    ok = qEKF->updateFilterWithMeasurements(linAcc, gyro);
    ASSERT_IS_TRUE(ok);

    x0(0) = 1.0;
    x0(1) = 0.0;
    x0(2) = 0.0;
    x0(3) = 0.0;
    iDynTree::Span<double> x1_span(x0.data(), x0.size());
    std::cout << "Setting initial internal state" << std::endl;
    ok = qEKF->setInternalState(x1_span);

    iDynTree::IAttitudeEstimator* qekf_(qEKF.get());
    for (int i = 0; i <10 ; i++ )
    {
        run(qekf_, linAcc, gyro, mag);
    }

    linAcc.zero();
    linAcc(2) = -9.8;
    gyro(1) = 0.0;

    for (int i = 0; i <10 ; i++ )
    {
        run(qekf_, linAcc, gyro, mag);
    }

    std::cout << "\nQuaternion EKF runs without faults." << std::endl;

    std::cout << "\n\n Mahony filter running..." << std::endl;

    std::unique_ptr<iDynTree::AttitudeMahonyFilter> mahony_filt;

    mahony_filt = std::make_unique<iDynTree::AttitudeMahonyFilter>();
    iDynTree::AttitudeMahonyFilterParameters mahony_params;
    mahony_params.kp = 0.7;
    mahony_params.ki = 0.01;
    mahony_params.time_step_in_seconds = 0.01;
    mahony_params.use_magnetometer_measurements = false;

    mahony_filt->setParameters(mahony_params);
    mahony_filt->setInternalState(x1_span);

    iDynTree::IAttitudeEstimator* mahony_(mahony_filt.get());
    for (int i = 0; i <10 ; i++ )
    {
        run(mahony_, linAcc, gyro, mag);
    }

    return EXIT_SUCCESS;
}
