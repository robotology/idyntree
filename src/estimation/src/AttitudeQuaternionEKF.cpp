/*
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Estimation/AttitudeQuaternionEKF.h>

void iDynTree::AttitudeQuaternionEKF::serializeStateVector()
{
    if (m_x.size() != m_state_size)
    {
        m_x.resize(m_state_size);
    }

    iDynTree::toEigen(m_x).block<4, 1>(0, 0) = iDynTree::toEigen(m_state.m_orientation);
    iDynTree::toEigen(m_x).block<3, 1>(4, 0) = iDynTree::toEigen(m_state.m_angular_velocity);
    iDynTree::toEigen(m_x).block<3, 1>(7, 0) = iDynTree::toEigen(m_state.m_gyroscope_bias);
}

void iDynTree::AttitudeQuaternionEKF::deserializeStateVector()
{
    iDynTree::toEigen(m_state.m_orientation) = iDynTree::toEigen(m_x).block<4, 1>(0, 0);
    iDynTree::toEigen(m_state.m_angular_velocity) = iDynTree::toEigen(m_x).block<3, 1>(4, 0);
    iDynTree::toEigen(m_state.m_gyroscope_bias) = iDynTree::toEigen(m_x).block<3, 1>(4, 0);
}

void iDynTree::AttitudeQuaternionEKF::prepareSystemNoiseCovarianceMatrix(iDynTree::MatrixDynSize &Q)
{
    if (Q.rows() != m_state_size && Q.cols() != m_state_size)
    {
        Q.resize(m_state_size, m_state_size);
    }

    iDynTree::MatrixDynSize Fu(m_state_size, m_input_size + m_state.m_gyroscope_bias.size());
    Fu.zero();
    iDynTree::toEigen(Fu).block<3, 3>(4, 0) = iDynTree::toEigen(m_Id3);
    iDynTree::toEigen(Fu).block<3,3>(7, 3) = iDynTree::toEigen(m_Id3);

    iDynTree::Matrix6x6 U;
    U.zero();
    iDynTree::toEigen(U).block<3, 3>(0, 0) = iDynTree::toEigen(m_Id3)*m_params.gyroscope_noise_variance;
    iDynTree::toEigen(U).block<3, 3>(3, 3) = iDynTree::toEigen(m_Id3)*m_params.gyro_bias_noise_variance;

    iDynTree::toEigen(Q) = iDynTree::toEigen(Fu)*iDynTree::toEigen(U)*iDynTree::toEigen(Fu).transpose();
}

void iDynTree::AttitudeQuaternionEKF::prepareMeasurementNoiseCovarianceMatrix(iDynTree::MatrixDynSize& R)
{
    if (R.rows() != m_output_size && R.cols() != m_output_size)
    {
        R.resize(m_output_size, m_output_size);
    }

    iDynTree::toEigen(R).block<3, 3>(0, 0) = iDynTree::toEigen(m_Id3)*m_params.accelerometer_noise_variance;
    if (m_params.use_magenetometer_measurements)
    {
        iDynTree::toEigen(R).block<1, 1>(3, 3) = iDynTree::toEigen(m_params.magnetometer_noise_variance);
    }
}

iDynTree::AttitudeQuaternionEKF::AttitudeQuaternionEKF()
{
      m_state.m_orientation.zero();
      m_state.m_orientation(0) = 1.0;
      m_state.m_angular_velocity.zero();
      m_state.m_gyroscope_bias.zero();
      m_initial_state = m_state;

      m_orientationInSO3.fromQuaternion(m_state.m_orientation);
      m_orientationInRPY = m_orientationInSO3.asRPY();

      m_Omega_y.zero();
      m_Acc_y.zero();

      iDynTree::toEigen(m_Id4).Identity();
      iDynTree::toEigen(m_Id3).Identity();
}

bool iDynTree::AttitudeQuaternionEKF::initializeFilter()
{
    m_state_size = getInternalStateSize();
    if (m_params.use_magenetometer_measurements)
    {
        m_output_size = output_dimensions_with_magnetometer;
    }
    else
    {
        m_output_size = output_dimensions_without_magnetometer;
    }
    ekfSetStateSize(m_state_size);
    serializeStateVector();
    ekfSetOutputSize(m_output_size);
    m_input_size = input_dimensions;
    ekfSetInputSize(m_input_size);
    m_y.resize(m_output_size);
    m_v.resize(m_output_size);
    m_w.resize(m_state_size);

    setSystemNoiseVariance(m_params.gyroscope_noise_variance,
                            m_params.gyro_bias_noise_variance);

    setMeasurementNoiseVariance(m_params.accelerometer_noise_variance,
                                m_params.magnetometer_noise_variance);

    if (!ekfInit())
    {
        return false;
    }

    return m_initialized;
}

bool iDynTree::AttitudeQuaternionEKF::propagateStates()
{
    iDynTree::Span<double> Omega_y_span(m_Omega_y.data(), m_Omega_y.size());
    ekfSetInputVector(Omega_y_span);
    ekfPredict();

    return true;
}

void iDynTree::AttitudeQuaternionEKF::serializeMeasurementVector()
{
    if (m_y.size() != m_output_size)
    {
        m_y.resize(m_output_size);
    }

    iDynTree::toEigen(m_y).block<3, 1>(0, 0) = iDynTree::toEigen(m_Acc_y);
    if (m_params.use_magenetometer_measurements)
    {
        iDynTree::toEigen(m_y).block<1, 1>(3, 0) = iDynTree::toEigen(m_Mag_y);
    }
}

bool iDynTree::AttitudeQuaternionEKF::callEkfUpdate()
{
    serializeMeasurementVector();
    iDynTree::Span<double> y_span(m_y.data(), m_y.size());
    ekfSetMeasurementVector(y_span);
    ekfUpdate();

    iDynTree::Span<double> x_span(m_x.data(), m_x.size());
    if (ekfGetStates(x_span))
    {
        deserializeStateVector();
    }
    else
    {
        iDynTree::reportError("AttitudeQuaternionEKF", "updateFilterWithMeasurements", "could not get recent state estimate");
        return false;
    }
    return true;
}


bool iDynTree::AttitudeQuaternionEKF::updateFilterWithMeasurements(const iDynTree::LinearAccelerometerMeasurements& linAccMeas, const iDynTree::GyroscopeMeasurements& gyroMeas, const iDynTree::MagnetometerMeasurements& magMeas)
{
    if (m_y.size() != m_output_size)
    {
        m_y.resize(m_output_size);
    }
    m_Acc_y = linAccMeas;
    iDynTree::toEigen(m_Acc_y).normalize();
    m_Omega_y = gyroMeas;
    // compute yaw angle from magnetometer measurements
    iDynTree::Vector3 mag_meas_in_inertial_frame;
    iDynTree::toEigen(mag_meas_in_inertial_frame) = iDynTree::toEigen(m_orientationInSO3) * iDynTree::toEigen(magMeas);
    mag_meas_in_inertial_frame(2) = 0; // to limit the vertical influence of magnetometer A^m_z = 0
    iDynTree::Vector3 modified_mag_meas_in_body_frame;
    iDynTree::toEigen(modified_mag_meas_in_body_frame) = iDynTree::toEigen(m_orientationInSO3).transpose()* iDynTree::toEigen(mag_meas_in_inertial_frame); // rotate vector back to body frame
    m_Mag_y = atan2(-modified_mag_meas_in_body_frame(1), modified_mag_meas_in_body_frame(0));

    // set accelerometer and magnetometer measurement

    if (!callEkfUpdate())
    {
        return false;
    }

    return true;
}

bool iDynTree::AttitudeQuaternionEKF::updateFilterWithMeasurements(const iDynTree::LinearAccelerometerMeasurements& linAccMeas, const iDynTree::GyroscopeMeasurements& gyroMeas)
{
    iDynTree::MagnetometerMeasurements magMeas;
    magMeas.zero();
    updateFilterWithMeasurements(linAccMeas, gyroMeas, magMeas);
    return true;
}

bool iDynTree::AttitudeQuaternionEKF::computejacobianF(iDynTree::VectorDynSize& x, iDynTree::MatrixDynSize& F)
{
    return true;
}

bool iDynTree::AttitudeQuaternionEKF::computejacobianH(iDynTree::VectorDynSize& x, iDynTree::MatrixDynSize& H)
{
    return true;
}

bool iDynTree::AttitudeQuaternionEKF::f(const iDynTree::VectorDynSize& x_k, const iDynTree::VectorDynSize& u_k, const iDynTree::VectorDynSize& w_k, iDynTree::VectorDynSize& xhat_k_plus_one)
{
    if (x_k.size() != m_x.size() || xhat_k_plus_one.size() != m_x.size())
    {
        reportError("AttitudeQuaternionEKF", "f", "state size mismatch");
        return false;
    }

    if (x_k.size() != m_u.size())
    {
        reportError("AttitudeQuaternionEKF", "f", "input size mismatch");
        return false;
    }

    if (w_k.size() != m_w.size())
    {
        // do nothing
    }

    iDynTree::Quaternion q;
    iDynTree::Vector3 ang_vel, gyro_bias;
    iDynTree::toEigen(q) = iDynTree::toEigen(x_k).block<4,1>(0, 0);
    iDynTree::toEigen(ang_vel) = iDynTree::toEigen(x_k).block<3,1>(4, 0);
    iDynTree::toEigen(gyro_bias) = iDynTree::toEigen(x_k).block<3,1>(7, 0);

    iDynTree::Quaternion correction = pureQuaternion(ang_vel);
    iDynTree::toEigen(xhat_k_plus_one).block<4,1>(0, 0) = iDynTree::toEigen(q) + iDynTree::toEigen((composeQuaternion2(q, correction)))*(m_params.time_step_in_seconds*0.5);
    iDynTree::toEigen(xhat_k_plus_one).block<3,1>(4, 0) = iDynTree::toEigen(u_k) - iDynTree::toEigen(gyro_bias);
    iDynTree::toEigen(xhat_k_plus_one).block<3,1>(7, 0) = iDynTree::toEigen(gyro_bias)*(1 - (m_params.bias_correlation_time_factor*m_params.time_step_in_seconds));

    // additive white noise - addition with zero mean
    //iDynTree::toEigen(xhat_k_plus_one) += iDynTree::toEigen(w_k);
    return true;
}

bool iDynTree::AttitudeQuaternionEKF::h(const iDynTree::VectorDynSize& xhat_k_plus_one, const iDynTree::VectorDynSize& v_k_plus_one, iDynTree::VectorDynSize& zhat_k_plus_one)
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

    if (v_k_plus_one.size() != m_v.size())
    {
        // do nothing
    }

    iDynTree::Quaternion q;
    iDynTree::toEigen(q) = iDynTree::toEigen(xhat_k_plus_one).block<4,1>(0, 0);
    double q1q3 = q(1)*q(3);
    double q0q2 = q(0)*q(2);
    double q2q3 = q(2)*q(3);
    double q0q1 = q(0)*q(1);
    double q0squared = q(0)*q(0);
    double q1squared = q(1)*q(1);
    double q2squared = q(2)*q(2);
    double q3squared = q(3)*q(3);

    zhat_k_plus_one(0) = -2*(q1q3 - q0q2);
    zhat_k_plus_one(1) = -2*(q2q3 + q0q1);
    zhat_k_plus_one(2) = -(q0squared - q1squared - q2squared + q3squared);

    if (m_output_size == output_dimensions_with_magnetometer)
    {
        double q0q3 = q(0)*q(3);
        double q1q2 = q(1)*q(2);
        zhat_k_plus_one(3) = atan2(2*(q0q3+q1q2), 1 - 2*(q2squared + q3squared));
    }

    return true;
}

void iDynTree::AttitudeQuaternionEKF::setMeasurementNoiseVariance(double acc, double mag)
{
    m_params.accelerometer_noise_variance = acc;
    m_params.magnetometer_noise_variance = mag;

    iDynTree::MatrixDynSize R = iDynTree::MatrixDynSize(m_output_size, m_output_size);
    prepareMeasurementNoiseCovarianceMatrix(R);
    iDynTree::Span<double> R_span(R.data(), R.capacity());
    iDynTree::Span<double> v_span(m_v.data(), m_v.size());
    ekfSetSystemNoiseMeanAndCovariance(v_span, R_span);
}

void iDynTree::AttitudeQuaternionEKF::setSystemNoiseVariance(double gyro, double gyro_bias)
{
    m_params.gyroscope_noise_variance = gyro;
    m_params.gyro_bias_noise_variance = gyro_bias;

    iDynTree::MatrixDynSize Q = iDynTree::MatrixDynSize(m_state_size, m_state_size);
    prepareSystemNoiseCovarianceMatrix(Q);
    iDynTree::Span<double> Q_span(Q.data(), Q.capacity());
    iDynTree::Span<double> w_span(m_w.data(), m_w.size());
    ekfSetSystemNoiseMeanAndCovariance(w_span, Q_span);
}

void iDynTree::AttitudeQuaternionEKF::setInitialStateCovariance(double orientation_var, double ang_vel_var, double gyro_bias_var)
{
    iDynTree::MatrixDynSize P(m_state_size, m_state_size);
    iDynTree::toEigen(P).block<4,4>(0,0) = iDynTree::toEigen(m_Id4)*orientation_var;
    iDynTree::toEigen(P).block<3,3>(4,4) = iDynTree::toEigen(m_Id3)*ang_vel_var;
    iDynTree::toEigen(P).block<3,3>(7,7) = iDynTree::toEigen(m_Id3)*gyro_bias_var;

    iDynTree::Span<double> P_span(P.data(), P.capacity());
    ekfSetStateCovariance(P_span);
}


bool iDynTree::AttitudeQuaternionEKF::getInternalInitialState(iDynTree::Span< double >& stateBuffer) const
{
    stateBuffer(0) = m_initial_state.m_orientation(0);
    stateBuffer(1) = m_initial_state.m_orientation(1);
    stateBuffer(2) = m_initial_state.m_orientation(2);
    stateBuffer(3) = m_initial_state.m_orientation(3);
    stateBuffer(4) = m_initial_state.m_angular_velocity(0);
    stateBuffer(5) = m_initial_state.m_angular_velocity(1);
    stateBuffer(6) = m_initial_state.m_angular_velocity(2);
    stateBuffer(7) = m_initial_state.m_gyroscope_bias(0);
    stateBuffer(8) = m_initial_state.m_gyroscope_bias(1);
    stateBuffer(9) = m_initial_state.m_gyroscope_bias(2);
    return true;
}

bool iDynTree::AttitudeQuaternionEKF::getInternalState(iDynTree::Span< double >& stateBuffer) const
{
    stateBuffer(0) = m_state.m_orientation(0);
    stateBuffer(1) = m_state.m_orientation(1);
    stateBuffer(2) = m_state.m_orientation(2);
    stateBuffer(3) = m_state.m_orientation(3);
    stateBuffer(4) = m_state.m_angular_velocity(0);
    stateBuffer(5) = m_state.m_angular_velocity(1);
    stateBuffer(6) = m_state.m_angular_velocity(2);
    stateBuffer(7) = m_state.m_gyroscope_bias(0);
    stateBuffer(8) = m_state.m_gyroscope_bias(1);
    stateBuffer(9) = m_state.m_gyroscope_bias(2);
    return true;
}

std::size_t iDynTree::AttitudeQuaternionEKF::getInternalStateSize() const
{
    size_t size = 0;
    size += m_state.m_orientation.size();
    size += m_state.m_angular_velocity.size();
    size += m_state.m_gyroscope_bias.size();
    return size;
}

bool iDynTree::AttitudeQuaternionEKF::getOrientationEstimateAsQuaternion(iDynTree::Quaternion& q)
{
    q = m_state.m_orientation;
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

bool iDynTree::AttitudeQuaternionEKF::setInternalState(iDynTree::Span< double >& stateBuffer)
{
    if ((size_t)stateBuffer.size() != getInternalStateSize())
    {
        iDynTree::reportError("AttitudeQuaternionEKF", "setInternalState", "state size mismatch, using default state");
        return false;
    }
    m_state.m_orientation(0) = stateBuffer(0);
    m_state.m_orientation(1) = stateBuffer(1);
    m_state.m_orientation(2) = stateBuffer(2);
    m_state.m_orientation(3) = stateBuffer(3);
    m_state.m_angular_velocity(0) = stateBuffer(4);
    m_state.m_angular_velocity(1) = stateBuffer(5);
    m_state.m_angular_velocity(2) = stateBuffer(6);
    m_state.m_gyroscope_bias(0) = stateBuffer(7);
    m_state.m_gyroscope_bias(1) = stateBuffer(8);
    m_state.m_gyroscope_bias(2) = stateBuffer(9);

    m_initial_state = m_state;
    serializeStateVector();
    iDynTree::Span<double> x0_span(m_x.data(), m_x.size());
    ekfSetInitialState(x0_span);
    return true;
}

void iDynTree::AttitudeQuaternionEKF::useMagnetometerMeasurements(bool use_magenetometer_measurements)
{
    m_params.use_magenetometer_measurements = use_magenetometer_measurements;
    if (m_params.use_magenetometer_measurements)
    {
        m_output_size = output_dimensions_with_magnetometer;
    }
    else
    {
        m_output_size = output_dimensions_without_magnetometer;
    }
}
