/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Estimation/AttitudeMahonyFilter.h>
#include <ctime>

iDynTree::Matrix3x3 getMatrixFromVectorVectorMultiplication(iDynTree::Vector3 a, iDynTree::Vector3 b)
{
    iDynTree::Matrix3x3 out;
    iDynTree::toEigen(out) = iDynTree::toEigen(a)*iDynTree::toEigen(b).transpose();
    return out;
}

iDynTree::Matrix3x3 getAngVelSkewSymmetricMatrixFromMeasurements(iDynTree::Vector3 meas, const iDynTree::Direction& vectorDir, double confidenceMeas, const iDynTree::Rotation& R)
{
    iDynTree::Direction va_hat = R.inverse().changeCoordFrameOf(vectorDir);
    iDynTree::Vector3 vectorial_estimate = iDynTree::Vector3(va_hat.data(), 3);
    iDynTree::toEigen(meas).normalize();
    iDynTree::Matrix3x3 A_acc = getMatrixFromVectorVectorMultiplication(meas, vectorial_estimate);
    iDynTree::Matrix3x3 S_acc;
    iDynTree::toEigen(S_acc) = (iDynTree::toEigen(A_acc) - iDynTree::toEigen(A_acc).transpose())*(confidenceMeas*0.5);
    return S_acc;
}

bool iDynTree::AttitudeMahonyFilter::updateFilterWithMeasurements(const iDynTree::LinearAccelerometerMeasurements& linAccMeas, const iDynTree::GyroscopeMeasurements& gyroMeas)
{
    iDynTree::Matrix3x3 S_acc = getAngVelSkewSymmetricMatrixFromMeasurements(linAccMeas, m_gravity_direction, 1-m_params.confidence_magnetometer_measurements, m_orientationInSO3);
    iDynTree::toEigen(m_omega_mes) = -iDynTree::toEigen(mapso3ToR3(S_acc));
    m_Omega_y = gyroMeas;
    return true;
}

bool iDynTree::AttitudeMahonyFilter::updateFilterWithMeasurements(const iDynTree::LinearAccelerometerMeasurements& linAccMeas, const iDynTree::GyroscopeMeasurements& gyroMeas, const iDynTree::MagnetometerMeasurements& magMeas)
{
    if (m_params.use_magnetometer_measurements == false)
    {
        iDynTree::reportWarning("AttitudeMahonyFilter", "updateFilterWithMeasurements", "useMagnetoMeterMeasurements set to false, using only accelerometer measurements");
        return updateFilterWithMeasurements(linAccMeas, gyroMeas);
    }

    iDynTree::Matrix3x3 S_acc = getAngVelSkewSymmetricMatrixFromMeasurements(linAccMeas, m_gravity_direction, 1-m_params.confidence_magnetometer_measurements, m_orientationInSO3);
    iDynTree::Matrix3x3 S_mag = getAngVelSkewSymmetricMatrixFromMeasurements(magMeas, m_earth_magnetic_field_direction, m_params.confidence_magnetometer_measurements, m_orientationInSO3);
    iDynTree::Matrix3x3 S_meas;
    iDynTree::toEigen(S_meas) = iDynTree::toEigen(S_acc) + iDynTree::toEigen(S_mag);

    iDynTree::toEigen(m_omega_mes) = -iDynTree::toEigen(mapso3ToR3(S_meas));
    m_Omega_y = gyroMeas;
    return true;
}

bool iDynTree::AttitudeMahonyFilter::propagateStates()
{
    iDynTree::Vector3 gyroUpdate;
    iDynTree::toEigen(gyroUpdate) = iDynTree::toEigen(m_Omega_y) - iDynTree::toEigen(m_state.m_gyroscope_bias) + (iDynTree::toEigen(m_omega_mes)*m_params.kp);

    iDynTree::Quaternion correction = pureQuaternion(gyroUpdate);
    iDynTree::toEigen(m_state.m_orientation) = iDynTree::toEigen(m_state.m_orientation) + iDynTree::toEigen((composeQuaternion2(m_state.m_orientation, correction)))*(m_params.time_step_in_seconds*0.5);
    iDynTree::toEigen(m_state.m_orientation).normalize();
    iDynTree::toEigen(m_state.m_angular_velocity) = iDynTree::toEigen(m_Omega_y) - iDynTree::toEigen(m_state.m_gyroscope_bias);
    iDynTree::toEigen(m_state.m_gyroscope_bias) = iDynTree::toEigen(m_state.m_gyroscope_bias) - ((iDynTree::toEigen(m_omega_mes)*m_params.ki)*(m_params.time_step_in_seconds));

    m_orientationInSO3 = iDynTree::Rotation::RotationFromQuaternion(m_state.m_orientation);
    //m_orientationInRPY = quaternion2eulerRPY(m_state.m_orientation);
    m_orientationInRPY = iDynTree::Rotation::RotationFromQuaternion(m_state.m_orientation).asRPY();
    return true;
}

iDynTree::AttitudeMahonyFilter::AttitudeMahonyFilter()
{
    m_params.time_step_in_seconds = 0.01;
    m_params.kp = 1.0;
    m_params.ki = 0.0;
    m_params.use_magnetometer_measurements = false;
    m_params.confidence_magnetometer_measurements = 0.0;

    m_state.m_orientation.zero();
    m_state.m_orientation(0) = 1.0;
    m_state.m_angular_velocity.zero();
    m_state.m_gyroscope_bias.zero();

    m_initial_state = m_state;

    m_gravity_direction.zero();
    m_gravity_direction(2) = 1.0; // pointing downwards

    m_earth_magnetic_field_direction.zero();
    m_earth_magnetic_field_direction(2) = 1.0;

    m_orientationInSO3.fromQuaternion(m_state.m_orientation);
    //m_orientationInRPY = quaternion2eulerRPY(m_state.m_orientation);
    m_orientationInRPY = iDynTree::Rotation::RotationFromQuaternion(m_state.m_orientation).asRPY();

    m_omega_mes.zero();
    m_Omega_y.zero();
}


void iDynTree::AttitudeMahonyFilter::setTimeStepInSeconds(double timestepInSeconds)
{
    m_params.time_step_in_seconds = timestepInSeconds;
}

void iDynTree::AttitudeMahonyFilter::setConfidenceForMagnetometerMeasurements(double confidence)
{
    if (m_params.use_magnetometer_measurements == false)
    {
        iDynTree::reportWarning("AttitudeMahonyFilter", "setConfidenceForMagnetometerMeasurements", "not using magnetometer measurements, setting confidence to zero.");
        m_params.confidence_magnetometer_measurements = 0.0;
    }
    m_params.confidence_magnetometer_measurements = confidence;
}

void iDynTree::AttitudeMahonyFilter::setGainki(double ki)
{
    m_params.ki = ki;
}

void iDynTree::AttitudeMahonyFilter::setGainkp(double kp)
{
    m_params.kp = kp;
}

void iDynTree::AttitudeMahonyFilter::useMagnetoMeterMeasurements(bool flag)
{
    m_params.use_magnetometer_measurements = flag;
    if (flag == false)
    {
        iDynTree::reportWarning("AttitudeMahonyFilter", "useMagnetoMeterMeasurements", "setting confidence on magnetometer measurements to zero.");
        m_params.confidence_magnetometer_measurements = 0.0;
    }
}

void iDynTree::AttitudeMahonyFilter::setGravityDirection(const iDynTree::Direction& gravity_dir)
{
    m_gravity_direction = gravity_dir;
}


bool iDynTree::AttitudeMahonyFilter::getInternalInitialState(iDynTree::Span< double >& stateBuffer) const
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

bool iDynTree::AttitudeMahonyFilter::getInternalState(iDynTree::Span< double >& stateBuffer) const
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

size_t iDynTree::AttitudeMahonyFilter::getInternalStateSize() const
{
    size_t size = 0;
    size += m_state.m_orientation.size();
    size += m_state.m_angular_velocity.size();
    size += m_state.m_gyroscope_bias.size();
    return size;
}

bool iDynTree::AttitudeMahonyFilter::getOrientationEstimateAsQuaternion(iDynTree::Quaternion& q)
{
    q = m_state.m_orientation;
    return true;
}

bool iDynTree::AttitudeMahonyFilter::getOrientationEstimateAsRotationMatrix(iDynTree::Rotation& rot)
{
    rot = m_orientationInSO3;
    return true;
}

bool iDynTree::AttitudeMahonyFilter::getOrientationEstimateAsRPY(iDynTree::RPY& rpy)
{
    rpy = m_orientationInRPY;
    return true;
}

void iDynTree::AttitudeMahonyFilter::getParameters(iDynTree::AttitudeMahonyFilterParameters& params)
{
    params = m_params;
}

bool iDynTree::AttitudeMahonyFilter::setInternalState(iDynTree::Span< double >& stateBuffer)
{
    if ((size_t)stateBuffer.size() != getInternalStateSize())
    {
        iDynTree::reportError("AttitudeMahonyFilter", "setInternalState", "state size mismatch, using default state");
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
    return true;
}

bool iDynTree::AttitudeMahonyFilter::setParameters(const iDynTree::AttitudeMahonyFilterParameters& params)
{   m_params = params;
    return true;
}
