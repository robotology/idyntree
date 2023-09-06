// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/AttitudeQuaternionEKF.h>
#include <iDynTree/AttitudeEstimatorUtils.h>

void iDynTree::AttitudeQuaternionEKF::serializeStateVector()
{
    using iDynTree::toEigen;
    auto x(toEigen(m_x));
    auto q(toEigen(m_state_qekf.m_orientation));
    auto Omega(toEigen(m_state_qekf.m_angular_velocity));
    auto b(toEigen(m_state_qekf.m_gyroscope_bias));

    if (m_x.size() != m_state_size)
    {
        m_x.resize(m_state_size);
    }

    x.block<4, 1>(0, 0) = q;
    x.block<3, 1>(4, 0) = Omega;
    x.block<3, 1>(7, 0) = b;
}

void iDynTree::AttitudeQuaternionEKF::deserializeStateVector()
{
    using iDynTree::toEigen;
    auto x(toEigen(m_x));
    auto q(toEigen(m_state_qekf.m_orientation));
    auto Omega(toEigen(m_state_qekf.m_angular_velocity));
    auto b(toEigen(m_state_qekf.m_gyroscope_bias));

    q = x.block<4, 1>(0, 0);
    Omega = x.block<3, 1>(4, 0);
    b = x.block<3, 1>(7, 0);
}

void iDynTree::AttitudeQuaternionEKF::prepareSystemNoiseCovarianceMatrix(iDynTree::MatrixDynSize &Q)
{
    using iDynTree::toEigen;

    if (Q.rows() != m_state_size && Q.cols() != m_state_size)
    {
        Q.resize(m_state_size, m_state_size);
    }


    iDynTree::MatrixDynSize Fu_dyn(m_state_size, m_input_size + m_state_qekf.m_gyroscope_bias.size());
    Fu_dyn.zero();
    iDynTree::Matrix6x6 U_dyn;
    U_dyn.zero();

    auto F_u(toEigen(Fu_dyn));
    auto U(toEigen(U_dyn));
    auto Id3(toEigen(m_Id3));
    auto Q_(toEigen(Q));

    F_u.block<3, 3>(4, 0) = Id3;
    F_u.block<3,3>(7, 3) = Id3;

    U.block<3, 3>(0, 0) = Id3*m_params_qekf.gyroscope_noise_variance;
    U.block<3, 3>(3, 3) = Id3*m_params_qekf.gyro_bias_noise_variance;

    Q_ = F_u*U*F_u.transpose();
}

void iDynTree::AttitudeQuaternionEKF::prepareMeasurementNoiseCovarianceMatrix(iDynTree::MatrixDynSize& R)
{
    using iDynTree::toEigen;

    if (R.rows() != m_output_size && R.cols() != m_output_size)
    {
        R.resize(m_output_size, m_output_size);
        R.zero();
    }

    auto R_(toEigen(R));
    auto Id3(toEigen(m_Id3));

    R_.block<3, 3>(0, 0) = Id3*m_params_qekf.accelerometer_noise_variance;
    if (m_params_qekf.use_magnetometer_measurements)
    {
        R_(3, 3) = m_params_qekf.magnetometer_noise_variance;
    }
}

iDynTree::AttitudeQuaternionEKF::AttitudeQuaternionEKF()
{
    m_state_qekf.m_orientation.zero();
    m_state_qekf.m_orientation(0) = 1.0;
    m_state_qekf.m_angular_velocity.zero();
    m_state_qekf.m_gyroscope_bias.zero();

    m_initial_state_qekf = m_state_qekf;
    m_orientationInSO3.fromQuaternion(m_state_qekf.m_orientation);
    m_orientationInRPY = m_orientationInSO3.asRPY();

    m_Omega_y.zero();
    m_Acc_y.zero();
    m_Acc_y(2) = 1; // TODO: validate this assumption at initial step

    m_gravity_direction.zero();
    m_gravity_direction(2) = 1;

    using iDynTree::toEigen;
    toEigen(m_Id4).setIdentity();
    toEigen(m_Id3).setIdentity();
}

bool iDynTree::AttitudeQuaternionEKF::initializeFilter()
{
    m_state_size = this->getInternalStateSize();

    if (m_params_qekf.use_magnetometer_measurements)
    {
        m_output_size = output_dimensions_with_magnetometer;
    }
    else
    {
        m_output_size = output_dimensions_without_magnetometer;
    }

    ekfSetStateSize(m_state_size);
    m_x.resize(m_state_size);
    serializeStateVector();
    ekfSetOutputSize(m_output_size);

    m_y.resize(m_output_size);
    m_input_size = input_dimensions;
    ekfSetInputSize(m_input_size);
    m_u.resize(m_input_size);

    // ekfinit resizes and initializes the filter matrices and vectors to zero - dont move it frome here
    if (!ekfInit())
    {
        return false;
    }

    // once the buffers are resized and zeroed, setup the matrices
    if (!setInitialStateCovariance(m_params_qekf.initial_orientation_error_variance,
                                   m_params_qekf.initial_ang_vel_error_variance,
                                   m_params_qekf.initial_gyro_bias_error_variance))
    {
        return false;
    }

    if (!setSystemNoiseVariance(m_params_qekf.gyroscope_noise_variance,
                                m_params_qekf.gyro_bias_noise_variance))
    {
        return false;
    }

    if (!setMeasurementNoiseVariance(m_params_qekf.accelerometer_noise_variance,
                                     m_params_qekf.magnetometer_noise_variance))
    {
        return false;
    }

    m_initialized = true;
    return m_initialized;
}

bool iDynTree::AttitudeQuaternionEKF::propagateStates()
{
    iDynTree::Span<double> Omega_y_span(m_Omega_y.data(), m_Omega_y.size());
    bool ok  = ekfSetInputVector(Omega_y_span);
    ok = ekfPredict() && ok;
    iDynTree::Span<double> x_span(m_x.data(), m_x.size());
    if (ekfGetStates(x_span))
    {
        deserializeStateVector();
        m_orientationInSO3 = iDynTree::Rotation::RotationFromQuaternion(m_state_qekf.m_orientation);
        m_orientationInRPY = m_orientationInSO3.asRPY();
    }
    else
    {
        iDynTree::reportError("AttitudeQuaternionEKF", "updateFilterWithMeasurements", "could not get recent state estimate");
        return false;
    }
    return ok;
}

void iDynTree::AttitudeQuaternionEKF::serializeMeasurementVector()
{
    using iDynTree::toEigen;
    if (m_y.size() != m_output_size)
    {
        m_y.resize(m_output_size);
    }

    toEigen(m_y).block<3, 1>(0, 0) = toEigen(m_Acc_y);
    if (m_params_qekf.use_magnetometer_measurements)
    {
        m_y(3) = m_Mag_y;
    }
}

bool iDynTree::AttitudeQuaternionEKF::callEkfUpdate()
{
    serializeMeasurementVector();
    iDynTree::Span<double> y_span(m_y.data(), m_y.size());
    bool ok = ekfSetMeasurementVector(y_span);
    ok = ekfUpdate() && ok;

    iDynTree::Span<double> x_span(m_x.data(), m_x.size());
    if (ekfGetStates(x_span))
    {
        deserializeStateVector();
        m_orientationInSO3 = iDynTree::Rotation::RotationFromQuaternion(m_state_qekf.m_orientation);
        m_orientationInRPY = m_orientationInSO3.asRPY();
    }
    else
    {
        iDynTree::reportError("AttitudeQuaternionEKF", "updateFilterWithMeasurements", "could not get recent state estimate");
        return false;
    }
    return ok;
}


bool iDynTree::AttitudeQuaternionEKF::updateFilterWithMeasurements(const iDynTree::LinearAccelerometerMeasurements& linAccMeas, const iDynTree::GyroscopeMeasurements& gyroMeas, const iDynTree::MagnetometerMeasurements& magMeas)
{
    if (m_y.size() != m_output_size)
    {
        m_y.resize(m_output_size);
    }

    if (!checkValidMeasurement(linAccMeas, "linear acceleration", true)) { return false; }
    if (!checkValidMeasurement(gyroMeas, "gyroscope", false)) { return false; }
    if (!checkValidMeasurement(magMeas, "magnetometer", true)) { return false; }

    iDynTree::Vector3 linAccMeasUnitVector;
    if (!getUnitVector(linAccMeas, linAccMeasUnitVector))
    {
        iDynTree::reportError("AttitudeQuaternionEKF", "updateFilterWithMeasurements", "Cannot retrieve unit vector from linear acceleration measuremnts.");
        return false;
    }

    m_Acc_y = linAccMeasUnitVector;
    m_Omega_y = gyroMeas;

    // compute yaw angle from magnetometer measurements by limiting the vertical influence of magentometer
    // to do this first rotate the measurement to inertial frame, set z-component to zero.
    // convert back to body frame and compute the yaw using atan2(y, x)
    iDynTree::Vector3 mag_meas_in_inertial_frame;
    iDynTree::Vector3 modified_mag_meas_in_body_frame;
    iDynTree::Vector3 magMeasUnitVector;
    if (!getUnitVector(magMeas, magMeasUnitVector))
    {
        iDynTree::reportError("AttitudeQuaternionEKF", "updateFilterWithMeasurements", "Cannot retrieve unit vector from magnetometer measuremnts.");
        return false;
    }

    using iDynTree::toEigen;
    auto m_A(toEigen(mag_meas_in_inertial_frame));
    auto m_B_modified(toEigen(modified_mag_meas_in_body_frame));
    auto m_yB(toEigen(magMeasUnitVector));
    auto A_R_B(toEigen(m_orientationInSO3));

    m_A = A_R_B * m_yB;
    m_A(2) = 0; // to limit the vertical influence of magnetometer {^A}m_z = 0
    m_B_modified = A_R_B.transpose()* m_A; // rotate vector back to body frame
    m_Mag_y = std::atan2(-m_B_modified(1), m_B_modified(0));

    // set accelerometer and magnetometer measurement
    if (!callEkfUpdate())
    {
        return false;
    }

    return true;
}

bool iDynTree::AttitudeQuaternionEKF::updateFilterWithMeasurements(const iDynTree::LinearAccelerometerMeasurements& linAccMeas, const iDynTree::GyroscopeMeasurements& gyroMeas)
{
    iDynTree::MagnetometerMeasurements magMeas; // dummy measurement
    magMeas.zero();
    magMeas(2) = 1;
    bool ok = updateFilterWithMeasurements(linAccMeas, gyroMeas, magMeas);
    return ok;
}

bool iDynTree::AttitudeQuaternionEKF::ekfComputeJacobianF(iDynTree::VectorDynSize& x, iDynTree::VectorDynSize& u, iDynTree::MatrixDynSize& F)
{
    ignore(u);
    ignore(x);
    ignore(F);
    return false;
}


bool iDynTree::AttitudeQuaternionEKF::ekfComputeJacobianF(iDynTree::VectorDynSize& x, iDynTree::MatrixDynSize& F)
{
    using iDynTree::toEigen;

    if (x.size() != m_state_size)
    {
        reportError("AttitudeQuaternionEKF", "computejacobianF", "state size mismatch");
        return false;
    }

    if (F.rows() != m_state_size || F.cols() != m_state_size)
    {
        reportError("AttitudeQuaternionEKF", "computejacobianF", "jacobian matrix size mismatch");
        return false;
    }

    F.zero();

    iDynTree::UnitQuaternion q;
    iDynTree::Vector3 ang_vel, gyro_bias;
    toEigen(q) = toEigen(x).block<4,1>(0, 0);
    toEigen(ang_vel) = toEigen(x).block<3,1>(4, 0);
    toEigen(gyro_bias) = toEigen(x).block<3,1>(7, 0);

    iDynTree::Matrix4x4 dfq_by_dq;
    dfq_by_dq(0,0) = dfq_by_dq(1, 1) = dfq_by_dq(2, 2) = dfq_by_dq(3, 3) = (2.0/m_params_qekf.time_step_in_seconds);
    dfq_by_dq(1, 0) = dfq_by_dq(2, 3) = ang_vel(0);
    dfq_by_dq(2, 0) = dfq_by_dq(3, 1) = ang_vel(1);
    dfq_by_dq(3, 0) = dfq_by_dq(1, 2) = ang_vel(2);
    dfq_by_dq(0, 1) = dfq_by_dq(3, 2) = -ang_vel(0);
    dfq_by_dq(0, 2) = dfq_by_dq(1, 3) = -ang_vel(1);
    dfq_by_dq(0, 3) = dfq_by_dq(2, 1) = -ang_vel(2);
    toEigen(dfq_by_dq) *= (m_params_qekf.time_step_in_seconds/2.0);

    iDynTree::MatrixDynSize dfq_by_dangvel;
    dfq_by_dangvel.resize(4, 3);
    dfq_by_dangvel(0, 0) = dfq_by_dangvel(2, 2) = -q(1);
    dfq_by_dangvel(0, 1) = dfq_by_dangvel(3, 0) = -q(2);
    dfq_by_dangvel(0, 2) = dfq_by_dangvel(1, 1) = -q(3);
    dfq_by_dangvel(2, 0) = q(3);
    dfq_by_dangvel(1, 2) = q(2);
    dfq_by_dangvel(3, 1) = q(1);
    dfq_by_dangvel(1, 0) = dfq_by_dangvel(2, 1) = dfq_by_dangvel(3, 2) = -q(0);
    toEigen(dfq_by_dangvel) *= (m_params_qekf.time_step_in_seconds/2.0);

    iDynTree::Matrix3x3 dfangvel_by_dgyrobias(m_Id3);
    toEigen(dfangvel_by_dgyrobias) *= -1;

    iDynTree::Matrix3x3 dfgyrobias_by_dgyrobias(m_Id3);
    toEigen(dfgyrobias_by_dgyrobias) *= (1 - (m_params_qekf.bias_correlation_time_factor*m_params_qekf.time_step_in_seconds));

    toEigen(F).block<4,4>(0,0) = toEigen(dfq_by_dq);
    toEigen(F).block<4,3>(0,4) = toEigen(dfq_by_dangvel);
    toEigen(F).block<3,3>(4,7) = toEigen(dfangvel_by_dgyrobias);
    toEigen(F).block<3,3>(7,7) = toEigen(dfgyrobias_by_dgyrobias);

    return true;
}

bool iDynTree::AttitudeQuaternionEKF::ekfComputeJacobianH(iDynTree::VectorDynSize& x, iDynTree::MatrixDynSize& H)
{
    using iDynTree::toEigen;

    if (x.size() != m_state_size)
    {
        reportError("AttitudeQuaternionEKF", "computejacobianH", "state size mismatch");
        return false;
    }

    if (H.rows() != m_output_size || H.cols() != m_state_size)
    {
        reportError("AttitudeQuaternionEKF", "computejacobianH", "jacobian matrix size mismatch");
        return false;
    }

    H.zero();
    iDynTree::UnitQuaternion q;
    toEigen(q) = toEigen(x).block<4,1>(0, 0);

    iDynTree::MatrixDynSize dhacc_by_dq;
    dhacc_by_dq.resize(3, 4);
    dhacc_by_dq(0, 0) = dhacc_by_dq(2, 2) = q(2);
    dhacc_by_dq(0, 1) = dhacc_by_dq(1, 2) = -q(3);
    dhacc_by_dq(0, 3) = dhacc_by_dq(1, 0) = -q(1);
    dhacc_by_dq(1, 1) = dhacc_by_dq(2, 0) = -q(0);
    dhacc_by_dq(0, 2) = q(0);
    dhacc_by_dq(1, 3) = -q(2);
    dhacc_by_dq(2, 1) = q(1);
    dhacc_by_dq(2, 3) = -q(3);

    if (m_gravity_direction(2) == -1)
    {
        toEigen(dhacc_by_dq) *= 2;
    }
    else if (m_gravity_direction(2) == 1)
    {
        toEigen(dhacc_by_dq) *= -2;
    }
    else
    {
        reportError("AttitudeQuaternionEKF", "computejacobianH", "filter assumes gravity pointing upward or downward only");
        return false;
    }

    toEigen(H).block<3, 4>(0, 0) = toEigen(dhacc_by_dq);

    if (m_output_size == output_dimensions_with_magnetometer)
    {
        double q0q3{q(0)*q(3)};
        double q1q2{q(1)*q(2)};
        double q2squared{q(2)*q(2)};
        double q3squared{q(3)*q(3)};
        double common_factor{(1 - 2*(q2squared + q3squared))};
        double common_factorSquared{common_factor*common_factor};
        double q0q3plusq1q2Squared{((q0q3+q1q2)*(q0q3+q1q2))};
        double denominator{(4*q0q3plusq1q2Squared) + (common_factorSquared)};

        double multiple{(2*common_factor/denominator)};
        double factor{(-8*(q0q3 + q1q2))/denominator};

        H(3, 0) = q(3)*multiple;
        H(3, 1) = q(2)*multiple;
        H(3, 1) = (q(1)*multiple) - (q(2)*factor);
        H(3, 2) = (q(0)*multiple) - (q(3)*factor);
    }

    return true;
}

bool iDynTree::AttitudeQuaternionEKF::ekf_f(const iDynTree::VectorDynSize& x_k, const iDynTree::VectorDynSize& u_k, iDynTree::VectorDynSize& xhat_k_plus_one)
{
    if (x_k.size() != m_x.size() || xhat_k_plus_one.size() != m_x.size())
    {
        reportError("AttitudeQuaternionEKF", "ekf_f", "state size mismatch");
        return false;
    }

    if (u_k.size() != m_u.size())
    {
        reportError("AttitudeQuaternionEKF", "ekf_f", "input size mismatch");
        return false;
    }

    iDynTree::UnitQuaternion orientation;
    iDynTree::Vector3 ang_vel, gyro_bias;

    using iDynTree::toEigen;
    auto q(toEigen(orientation));
    auto Omega(toEigen(ang_vel));
    auto b(toEigen(gyro_bias));
    auto x(toEigen(x_k));
    auto u(toEigen(u_k));
    auto x_hat_plus(toEigen(xhat_k_plus_one));

    q = x.block<4,1>(0, 0);
    Omega = x.block<3,1>(4, 0)*m_params_qekf.time_step_in_seconds;
    b = x.block<3,1>(7, 0);

    iDynTree::UnitQuaternion correction = expQuaternion(ang_vel);
    iDynTree::UnitQuaternion q_hat_plus = composeQuaternion2(orientation, correction);

    x_hat_plus.block<4,1>(0, 0) = toEigen(q_hat_plus);
    double quat_norm = x_hat_plus.block<4,1>(0, 0).norm();

    double malformed_unit_quaternion_norm{0.0};
    if (check_are_almost_equal(quat_norm, malformed_unit_quaternion_norm, 4))
    {
        reportError("AttitudeQuaternionEKF", "ekf_f", "invalid quaternion");
        return false;
    }

    double unit_quaternion_norm{1.0};
    if (!check_are_almost_equal(quat_norm, unit_quaternion_norm, 4))
    {
        x_hat_plus.block<4,1>(0, 0).normalize();
    }
    x_hat_plus.block<3,1>(4, 0) = u - b;
    x_hat_plus.block<3,1>(7, 0) = b*(1 - (m_params_qekf.bias_correlation_time_factor*m_params_qekf.time_step_in_seconds));

    return true;
}

bool iDynTree::AttitudeQuaternionEKF::ekf_h(const iDynTree::VectorDynSize& xhat_k_plus_one, iDynTree::VectorDynSize& zhat_k_plus_one)
{
    if (xhat_k_plus_one.size() != m_x.size())
    {
        reportError("AttitudeQuaternionEKF", "h", "predicted state size mismatch");
        return false;
    }

    if (zhat_k_plus_one.size() != m_y.size())
    {
        reportError("AttitudeQuaternionEKF", "h", "output size mismatch");
        return false;
    }

    // following computation is the same as R^T e_3
    iDynTree::UnitQuaternion q;
    toEigen(q) = toEigen(xhat_k_plus_one).block<4,1>(0, 0);
    double q1q3{q(1)*q(3)};
    double q0q2{q(0)*q(2)};
    double q2q3{q(2)*q(3)};
    double q0q1{q(0)*q(1)};
    double q0squared{q(0)*q(0)};
    double q1squared{q(1)*q(1)};
    double q2squared{q(2)*q(2)};
    double q3squared{q(3)*q(3)};

    zhat_k_plus_one(0) = -2*(q1q3 - q0q2);
    zhat_k_plus_one(1) = -2*(q2q3 + q0q1);
    zhat_k_plus_one(2) = -(q0squared - q1squared - q2squared + q3squared);
    if (m_gravity_direction(2) == -1)
    {
        // have same zhat
    }
    else if (m_gravity_direction(2) == 1)
    {
        toEigen(zhat_k_plus_one) *= -1;
    }
    else
    {
        reportError("AttitudeQuaternionEKF", "h", "filter assumes gravity pointing upward or downward only");
        return false;
    }

    // magnetometer measurement gives us yaw
    if (m_output_size == output_dimensions_with_magnetometer)
    {
        double q0q3{q(0)*q(3)};
        double q1q2{q(1)*q(2)};
        zhat_k_plus_one(3) = std::atan2(2*(q0q3+q1q2), 1 - 2*(q2squared + q3squared));
    }

    return true;
}

bool iDynTree::AttitudeQuaternionEKF::setMeasurementNoiseVariance(double acc, double mag)
{
    m_params_qekf.accelerometer_noise_variance = acc;
    m_params_qekf.magnetometer_noise_variance = mag;

    iDynTree::MatrixDynSize R = iDynTree::MatrixDynSize(m_output_size, m_output_size);
    prepareMeasurementNoiseCovarianceMatrix(R);

    iDynTree::Span<double> R_span(R.data(), R.capacity());
    bool ok = ekfSetMeasurementNoiseCovariance(R_span);
    return ok;
}

bool iDynTree::AttitudeQuaternionEKF::setSystemNoiseVariance(double gyro, double gyro_bias)
{
    m_params_qekf.gyroscope_noise_variance = gyro;
    m_params_qekf.gyro_bias_noise_variance = gyro_bias;

    iDynTree::MatrixDynSize Q = iDynTree::MatrixDynSize(m_state_size, m_state_size);
    prepareSystemNoiseCovarianceMatrix(Q);
    iDynTree::Span<double> Q_span(Q.data(), Q.capacity());
    bool ok = ekfSetSystemNoiseCovariance(Q_span);
    return ok;
}

bool iDynTree::AttitudeQuaternionEKF::setInitialStateCovariance(double orientation_var, double ang_vel_var, double gyro_bias_var)
{
    using iDynTree::toEigen;

    iDynTree::MatrixDynSize P(m_state_size, m_state_size);

    auto P_eig(toEigen(P));
    auto Id3(toEigen(m_Id3));
    auto Id4(toEigen(m_Id4));

    P_eig.block<4,4>(0,0) = Id4*orientation_var;
    P_eig.block<3,3>(4,4) = Id3*ang_vel_var;
    P_eig.block<3,3>(7,7) = Id3*gyro_bias_var;

    iDynTree::Span<double> P_span(P.data(), P.capacity());
    bool ok = ekfSetStateCovariance(P_span);
    return ok;
}


bool iDynTree::AttitudeQuaternionEKF::getDefaultInternalInitialState(const iDynTree::Span< double >& stateBuffer) const
{
    stateBuffer(0) = m_initial_state_qekf.m_orientation(0);
    stateBuffer(1) = m_initial_state_qekf.m_orientation(1);
    stateBuffer(2) = m_initial_state_qekf.m_orientation(2);
    stateBuffer(3) = m_initial_state_qekf.m_orientation(3);
    stateBuffer(4) = m_initial_state_qekf.m_angular_velocity(0);
    stateBuffer(5) = m_initial_state_qekf.m_angular_velocity(1);
    stateBuffer(6) = m_initial_state_qekf.m_angular_velocity(2);
    stateBuffer(7) = m_initial_state_qekf.m_gyroscope_bias(0);
    stateBuffer(8) = m_initial_state_qekf.m_gyroscope_bias(1);
    stateBuffer(9) = m_initial_state_qekf.m_gyroscope_bias(2);
    return true;
}

bool iDynTree::AttitudeQuaternionEKF::getInternalState(const iDynTree::Span< double >& stateBuffer) const
{
    stateBuffer(0) = m_state_qekf.m_orientation(0);
    stateBuffer(1) = m_state_qekf.m_orientation(1);
    stateBuffer(2) = m_state_qekf.m_orientation(2);
    stateBuffer(3) = m_state_qekf.m_orientation(3);
    stateBuffer(4) = m_state_qekf.m_angular_velocity(0);
    stateBuffer(5) = m_state_qekf.m_angular_velocity(1);
    stateBuffer(6) = m_state_qekf.m_angular_velocity(2);
    stateBuffer(7) = m_state_qekf.m_gyroscope_bias(0);
    stateBuffer(8) = m_state_qekf.m_gyroscope_bias(1);
    stateBuffer(9) = m_state_qekf.m_gyroscope_bias(2);
    return true;
}

std::size_t iDynTree::AttitudeQuaternionEKF::getInternalStateSize() const
{
    size_t size = 0;
    size += m_state_qekf.m_orientation.size();
    size += m_state_qekf.m_angular_velocity.size();
    size += m_state_qekf.m_gyroscope_bias.size();

    return size;
}

bool iDynTree::AttitudeQuaternionEKF::getOrientationEstimateAsQuaternion(iDynTree::UnitQuaternion& q)
{
    q = m_state_qekf.m_orientation;
    return true;
}

bool iDynTree::AttitudeQuaternionEKF::getOrientationEstimateAsRotationMatrix(iDynTree::Rotation& rot)
{
    rot = m_orientationInSO3;
    return true;
}

bool iDynTree::AttitudeQuaternionEKF::getOrientationEstimateAsRPY(iDynTree::RPY& rpy)
{
    rpy = m_orientationInRPY;
    return true;
}

bool iDynTree::AttitudeQuaternionEKF::setInternalState(const iDynTree::Span< double >& stateBuffer)
{
    if ((size_t)stateBuffer.size() != getInternalStateSize())
    {
        iDynTree::reportError("AttitudeQuaternionEKF", "setInternalState", "state size mismatch, using default state");
        return false;
    }
    m_state_qekf.m_orientation(0) = stateBuffer(0);
    m_state_qekf.m_orientation(1) = stateBuffer(1);
    m_state_qekf.m_orientation(2) = stateBuffer(2);
    m_state_qekf.m_orientation(3) = stateBuffer(3);
    m_state_qekf.m_angular_velocity(0) = stateBuffer(4);
    m_state_qekf.m_angular_velocity(1) = stateBuffer(5);
    m_state_qekf.m_angular_velocity(2) = stateBuffer(6);
    m_state_qekf.m_gyroscope_bias(0) = stateBuffer(7);
    m_state_qekf.m_gyroscope_bias(1) = stateBuffer(8);
    m_state_qekf.m_gyroscope_bias(2) = stateBuffer(9);

    m_initial_state_qekf = m_state_qekf;
    serializeStateVector();
    iDynTree::Span<double> x0_span(m_x.data(), m_x.size());
    bool ok = ekfSetInitialState(x0_span);
    return ok;
}

bool iDynTree::AttitudeQuaternionEKF::setInternalStateInitialOrientation(const iDynTree::Span< double >& orientationBuffer)
{
    if ((size_t)orientationBuffer.size() != m_state_qekf.m_orientation.size())
    {
        iDynTree::reportError("AttitudeQuaternionEKF", "setInternalStateInitialOrientation", "orientation size mismatch, using default state");
        return false;
    }

    m_state_qekf.m_orientation(0) = orientationBuffer(0);
    m_state_qekf.m_orientation(1) = orientationBuffer(1);
    m_state_qekf.m_orientation(2) = orientationBuffer(2);
    m_state_qekf.m_orientation(3) = orientationBuffer(3);

    m_initial_state_qekf = m_state_qekf;
    serializeStateVector();
    iDynTree::Span<double> x0_span(m_x.data(), m_x.size());
    bool ok = ekfSetInitialState(x0_span);

    return ok;
}


bool iDynTree::AttitudeQuaternionEKF::useMagnetometerMeasurements(bool use_magnetometer_measurements)
{
    if (use_magnetometer_measurements == m_params_qekf.use_magnetometer_measurements)
    {
        return true;
    }

    // store current state estimate and variance
    iDynTree::VectorDynSize x(m_x);
    iDynTree::MatrixDynSize P(m_x.size(), m_x.size());
    iDynTree::Span<double> P_span(P.data(), P.capacity());
    if (!ekfGetStateCovariance(P_span))
    {
        return false;
    }

    // get variances
    double orientation_var{P(0,0)};
    double ang_vel_var{P(4,4)};
    double gyro_bias_var{P(7,7)};

    m_params_qekf.use_magnetometer_measurements = use_magnetometer_measurements;
    ekfReset();

    bool ok = initializeFilter();
    iDynTree::Span<double> x_span(x.data(), x.size());
    ok = setInternalState(x_span) && ok;
    ok = setInitialStateCovariance(orientation_var, ang_vel_var, gyro_bias_var) && ok;
    return ok;
}

void iDynTree::AttitudeQuaternionEKF::setGravityDirection(const iDynTree::Direction& gravity_dir)
{
    m_gravity_direction = gravity_dir;
}

