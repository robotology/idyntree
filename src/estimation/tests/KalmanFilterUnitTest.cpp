// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/KalmanFilter.h>
#include <iDynTree/TestUtils.h>
#include <iostream>
#include <memory>

#include <random>

int main()
{
    iDynTree::DiscreteKalmanFilterHelper kf;
    // state space is position and velocity of truck with constant input acceleration u = a
    // x {k+1} = A x {k} + B u {k}
    // x1 {k+1} = x1 {k} + dt x2 {k} + (0.5) (dt^2) u
    // x2 {k+1} = x2 {k} + dt u

    double dt{1.0}; ///< discretization_time_step_in_s
    iDynTree::MatrixDynSize A;
    A.resize(2, 2);
    A(0, 0) = 1; A(0, 1) = dt;
    A(1, 0) = 0; A(1, 1) = 1;

    iDynTree::MatrixDynSize B;
    B.resize(2, 1);
    B(0, 0) = 0.5*dt*dt;
    B(1, 0) = dt;

    // noisy measurement of the truck is made at each step without any feedthrough
    iDynTree::MatrixDynSize C, D;
    C.resize(1, 2);
    C(0, 0) = 1; C(0, 1) = 2;

    bool ok{false};
    ok = kf.constructKalmanFilter(A, B, C);
    ASSERT_IS_TRUE(ok);
    std::cout << "Kalman filter constructed successfully." << std::endl;

    iDynTree::MatrixDynSize Q;
    Q.resize(2, 2);
    double system_noise_var{1.0};
    iDynTree::toEigen(Q) = system_noise_var*Eigen::MatrixXd::Identity(2, 2);
    ok = kf.kfSetSystemNoiseCovariance(Q);
    ASSERT_IS_TRUE(ok);
    std::cout << "System noise covariance matrix set." << std::endl;

    iDynTree::MatrixDynSize R;
    R.resize(1, 1);
    double measurement_noise_var{1.0};
    R(0, 0) = measurement_noise_var;
    ok = kf.kfSetMeasurementNoiseCovariance(R);
    ASSERT_IS_TRUE(ok);
    std::cout << "Measurement noise covariance matrix set." << std::endl;

    iDynTree::VectorDynSize x0;
    x0.resize(2);
    x0.zero();
    ok = kf.kfSetInitialState(x0);
    ASSERT_IS_TRUE(ok);
    std::cout << "initial state set." << std::endl;

    iDynTree::MatrixDynSize P0;
    P0.resize(2, 2);
    double state_var{1.0};
    iDynTree::toEigen(P0) = state_var*Eigen::MatrixXd::Identity(2, 2);
    ok = kf.kfSetStateCovariance(P0);
    std::cout << "initial state covariance set." << std::endl;

    ok = kf.kfInit();
    ASSERT_IS_TRUE(ok);
    std::cout << "Kalman filter initialized." << std::endl;

    // adding Gaussian noise to measurement
    // setting up constant seed for RNG
    // snippet taken from https://www.musicdsp.org/en/latest/Synthesis/168-c-gaussian-noise-generation.html
    std::srand(0.0);
    /* Setup constants */
    const static int q = 15;
    const static double c1 = (1 << q) - 1;
    const static double c2 = ((int)(c1 / 3)) + 1;
    const static double c3 = 1.f / c1;

    for (int i = 0; i <20 ; i++ )
    {
        std::cout << "\n\n\n Iteration: " << i << std::endl;
        iDynTree::VectorDynSize u;
        u.resize(1);
        u.zero();
        ok = kf.kfSetInputVector(u);
        ASSERT_IS_TRUE(ok);
        std::cout << "setting input." << std::endl;
        ok = kf.kfPredict();
        ASSERT_IS_TRUE(ok);
        std::cout << "kf prediction step ... passed." << std::endl;

        iDynTree::VectorDynSize x;
        kf.kfGetStates(x);

        iDynTree::VectorDynSize y;
        y.resize(1);
        y(0) = x(0);

        // adding gaussian noise
        // snippet taken from https://www.musicdsp.org/en/latest/Synthesis/168-c-gaussian-noise-generation.html
        double random = static_cast<double>(rand()) / (static_cast<double>(RAND_MAX) + 1);
        double noise = (2.f * ((random * c2) + (random * c2) + (random * c2)) - 3.f * (c2 - 1.f)) * c3;
        y(0) += noise;

        ok = kf.kfSetMeasurementVector(y);
        ASSERT_IS_TRUE(ok);
        std::cout << "setting noisy output." << std::endl;
        ok = kf.kfUpdate();
        ASSERT_IS_TRUE(ok);
        std::cout << "kf update step passed." << std::endl;

        iDynTree::VectorDynSize xhat;
        kf.kfGetStates(xhat);
        std::cout << "Estimated position: " << xhat(0) << " Estimated velocity: " << xhat(1) << std::endl;
    }

    return EXIT_SUCCESS;
}

