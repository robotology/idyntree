// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/AttitudeMahonyFilter.h>
#include <iDynTree/AttitudeEstimatorUtils.h>
#include <ctime>

iDynTree::Matrix3x3 getMatrixFromVectorVectorMultiplication(iDynTree::Vector3 a, iDynTree::Vector3 b)
{
    using iDynTree::toEigen;
    iDynTree::Matrix3x3 out;

    toEigen(out) = toEigen(a)*toEigen(b).transpose(); // to be read as out = a*(b.transpose())

    return out;
}

iDynTree::Matrix3x3 getAngVelSkewSymmetricMatrixFromMeasurements(iDynTree::Vector3 meas, const iDynTree::Direction& vectorDir, double confidenceMeas, const iDynTree::Rotation& R)
{
    using iDynTree::toEigen;

    ///< compute \f$ {{^A}{\hat{R}}_B}^T e_3 \f$ where \f$ e_3 \f$ is the vectorDir
    iDynTree::Direction va_hat = R.inverse().changeCoordFrameOf(vectorDir);
    iDynTree::Vector3 vectorial_estimate = iDynTree::Vector3(va_hat.data(), 3);

    ///< compute vectorial direction from the measurement normalized
    if (toEigen(meas).norm() != 1)
    {
        toEigen(meas).normalize();
    }

    iDynTree::Matrix3x3 Adyn = getMatrixFromVectorVectorMultiplication(meas, vectorial_estimate);
    iDynTree::Matrix3x3 Sdyn;
    auto A(toEigen(Adyn));
    auto S(toEigen(Sdyn));

    S = (A - A.transpose())*(confidenceMeas*0.5);

    return Sdyn;
}

bool iDynTree::AttitudeMahonyFilter::updateFilterWithMeasurements(const iDynTree::LinearAccelerometerMeasurements& linAccMeas, const iDynTree::GyroscopeMeasurements& gyroMeas)
{
    using iDynTree::toEigen;

    if (!checkValidMeasurement(linAccMeas, "linear acceleration", true)) { return false; }
    if (!checkValidMeasurement(gyroMeas, "gyroscope", false)) { return false; }

    iDynTree::Vector3 linAccMeasUnitVector;
    if (!getUnitVector(linAccMeas, linAccMeasUnitVector))
    {
        iDynTree::reportError("AttitudeMahonyFilter", "updateFilterWithMeasurements", "Cannot retrieve unit vector from linear acceleration measuremnts.");
        return false;
    }

    iDynTree::Matrix3x3 S_acc = getAngVelSkewSymmetricMatrixFromMeasurements(linAccMeasUnitVector, m_gravity_direction, 1-m_params_mahony.confidence_magnetometer_measurements, m_orientationInSO3);
    toEigen(m_omega_mes) = -toEigen(mapso3ToR3(S_acc));
    m_Omega_y = gyroMeas;

    return true;
}

bool iDynTree::AttitudeMahonyFilter::updateFilterWithMeasurements(const iDynTree::LinearAccelerometerMeasurements& linAccMeas, const iDynTree::GyroscopeMeasurements& gyroMeas, const iDynTree::MagnetometerMeasurements& magMeas)
{
    using iDynTree::toEigen;

    if (m_params_mahony.use_magnetometer_measurements == false)
    {
        return updateFilterWithMeasurements(linAccMeas, gyroMeas);
    }

    if (!checkValidMeasurement(linAccMeas, "linear acceleration", true)) { return false; }
    if (!checkValidMeasurement(gyroMeas, "gyroscope", false)) { return false; }
    if (!checkValidMeasurement(magMeas, "magnetometer", true)) { return false; }

    iDynTree::Vector3 linAccMeasUnitVector;
    if (!getUnitVector(linAccMeas, linAccMeasUnitVector))
    {
        iDynTree::reportError("AttitudeMahonyFilter", "updateFilterWithMeasurements", "Cannot retrieve unit vector from linear acceleration measuremnts.");
        return false;
    }

    iDynTree::Vector3 magMeasUnitVector;
    if (!getUnitVector(magMeas, magMeasUnitVector))
    {
        iDynTree::reportError("AttitudeMahonyFilter", "updateFilterWithMeasurements", "Cannot retrieve unit vector from magnetometer measuremnts.");
        return false;
    }

    iDynTree::Matrix3x3 S_acc = getAngVelSkewSymmetricMatrixFromMeasurements(linAccMeasUnitVector, m_gravity_direction, 1-m_params_mahony.confidence_magnetometer_measurements, m_orientationInSO3);
    iDynTree::Matrix3x3 S_mag = getAngVelSkewSymmetricMatrixFromMeasurements(magMeasUnitVector, m_earth_magnetic_field_direction, m_params_mahony.confidence_magnetometer_measurements, m_orientationInSO3);
    iDynTree::Matrix3x3 S_meas;

    toEigen(S_meas) = toEigen(S_acc) + toEigen(S_mag);
    toEigen(m_omega_mes) = -toEigen(mapso3ToR3(S_meas));
    m_Omega_y = gyroMeas;
    return true;
}

bool iDynTree::AttitudeMahonyFilter::propagateStates()
{
    using iDynTree::toEigen;
    iDynTree::Vector3 gyro_update_dyn;
    auto q(toEigen(m_state_mahony.m_orientation));
    auto Omega(toEigen(m_state_mahony.m_angular_velocity));
    auto b(toEigen(m_state_mahony.m_gyroscope_bias));
    auto gyroUpdate(toEigen(gyro_update_dyn));
    auto Omega_y(toEigen(m_Omega_y));
    auto omega_mes(toEigen(m_omega_mes));

    // compute the correction from the measurements
    gyroUpdate = (Omega_y - b + (omega_mes*m_params_mahony.kp))*m_params_mahony.time_step_in_seconds*0.5;
    iDynTree::UnitQuaternion correction = expQuaternion(gyro_update_dyn);

    // system dynamics equations
    q = toEigen(composeQuaternion2(m_state_mahony.m_orientation, correction));

    int precision{4};
    double malformed_unit_quaternion_norm{0.0};
    if (check_are_almost_equal(q.norm(), malformed_unit_quaternion_norm, precision))
    {
        reportError("AttitudeMahonyFilter", "propagateStates", "invalid quaternion with zero norm");
        return false;
    }

    double unit_quaternion_norm{1.0};
    if (!check_are_almost_equal(q.norm(), unit_quaternion_norm, precision))
    {
        q.normalize();
    }
    Omega = Omega_y - b;
    b = b - (omega_mes*m_params_mahony.ki)*(m_params_mahony.time_step_in_seconds);

    m_orientationInSO3 = iDynTree::Rotation::RotationFromQuaternion(m_state_mahony.m_orientation);
    m_orientationInRPY = iDynTree::Rotation::RotationFromQuaternion(m_state_mahony.m_orientation).asRPY();

    return true;
}

iDynTree::AttitudeMahonyFilter::AttitudeMahonyFilter()
{
    m_params_mahony.time_step_in_seconds = 0.01;
    m_params_mahony.kp = 1.0;
    m_params_mahony.ki = 0.0;
    m_params_mahony.use_magnetometer_measurements = false;
    m_params_mahony.confidence_magnetometer_measurements = 0.0;

    m_state_mahony.m_orientation.zero();
    m_state_mahony.m_orientation(0) = 1.0;
    m_state_mahony.m_angular_velocity.zero();
    m_state_mahony.m_gyroscope_bias.zero();

    m_initial_state_mahony = m_state_mahony;

    m_gravity_direction.zero();
    m_gravity_direction(2) = 1.0; // pointing downwards

    m_earth_magnetic_field_direction.zero();
    m_earth_magnetic_field_direction(2) = 1.0;

    m_orientationInSO3.fromQuaternion(m_state_mahony.m_orientation);
    m_orientationInRPY = iDynTree::Rotation::RotationFromQuaternion(m_state_mahony.m_orientation).asRPY();

    m_omega_mes.zero();
    m_Omega_y.zero();
}


void iDynTree::AttitudeMahonyFilter::setTimeStepInSeconds(double timestepInSeconds)
{
    m_params_mahony.time_step_in_seconds = timestepInSeconds;
}

void iDynTree::AttitudeMahonyFilter::setConfidenceForMagnetometerMeasurements(double confidence)
{
    if (m_params_mahony.use_magnetometer_measurements == false)
    {
        iDynTree::reportWarning("AttitudeMahonyFilter", "setConfidenceForMagnetometerMeasurements", "not using magnetometer measurements, setting confidence to zero.");
        m_params_mahony.confidence_magnetometer_measurements = 0.0;
    }
    m_params_mahony.confidence_magnetometer_measurements = confidence;
}

void iDynTree::AttitudeMahonyFilter::setGainki(double ki)
{
    m_params_mahony.ki = ki;
}

void iDynTree::AttitudeMahonyFilter::setGainkp(double kp)
{
    m_params_mahony.kp = kp;
}

void iDynTree::AttitudeMahonyFilter::useMagnetoMeterMeasurements(bool flag)
{
    m_params_mahony.use_magnetometer_measurements = flag;
    if (flag == false)
    {
        iDynTree::reportWarning("AttitudeMahonyFilter", "useMagnetoMeterMeasurements", "setting confidence on magnetometer measurements to zero.");
        m_params_mahony.confidence_magnetometer_measurements = 0.0;
    }
}

void iDynTree::AttitudeMahonyFilter::setGravityDirection(const iDynTree::Direction& gravity_dir)
{
    m_gravity_direction = gravity_dir;
}


bool iDynTree::AttitudeMahonyFilter::getDefaultInternalInitialState(const iDynTree::Span< double >& stateBuffer) const
{
    stateBuffer(0) = m_initial_state_mahony.m_orientation(0);
    stateBuffer(1) = m_initial_state_mahony.m_orientation(1);
    stateBuffer(2) = m_initial_state_mahony.m_orientation(2);
    stateBuffer(3) = m_initial_state_mahony.m_orientation(3);
    stateBuffer(4) = m_initial_state_mahony.m_angular_velocity(0);
    stateBuffer(5) = m_initial_state_mahony.m_angular_velocity(1);
    stateBuffer(6) = m_initial_state_mahony.m_angular_velocity(2);
    stateBuffer(7) = m_initial_state_mahony.m_gyroscope_bias(0);
    stateBuffer(8) = m_initial_state_mahony.m_gyroscope_bias(1);
    stateBuffer(9) = m_initial_state_mahony.m_gyroscope_bias(2);
    return true;
}

bool iDynTree::AttitudeMahonyFilter::getInternalState(const iDynTree::Span< double >& stateBuffer) const
{
    stateBuffer(0) = m_state_mahony.m_orientation(0);
    stateBuffer(1) = m_state_mahony.m_orientation(1);
    stateBuffer(2) = m_state_mahony.m_orientation(2);
    stateBuffer(3) = m_state_mahony.m_orientation(3);
    stateBuffer(4) = m_state_mahony.m_angular_velocity(0);
    stateBuffer(5) = m_state_mahony.m_angular_velocity(1);
    stateBuffer(6) = m_state_mahony.m_angular_velocity(2);
    stateBuffer(7) = m_state_mahony.m_gyroscope_bias(0);
    stateBuffer(8) = m_state_mahony.m_gyroscope_bias(1);
    stateBuffer(9) = m_state_mahony.m_gyroscope_bias(2);
    return true;
}

size_t iDynTree::AttitudeMahonyFilter::getInternalStateSize() const
{
    size_t size = 0;
    size += m_state_mahony.m_orientation.size();
    size += m_state_mahony.m_angular_velocity.size();
    size += m_state_mahony.m_gyroscope_bias.size();
    return size;
}

bool iDynTree::AttitudeMahonyFilter::getOrientationEstimateAsQuaternion(iDynTree::UnitQuaternion& q)
{
    q = m_state_mahony.m_orientation;
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

bool iDynTree::AttitudeMahonyFilter::setInternalState(const iDynTree::Span< double >& stateBuffer)
{
    if ((size_t)stateBuffer.size() != getInternalStateSize())
    {
        iDynTree::reportError("AttitudeMahonyFilter", "setInternalState", "state size mismatch, using default state");
        return false;
    }
    m_state_mahony.m_orientation(0) = stateBuffer(0);
    m_state_mahony.m_orientation(1) = stateBuffer(1);
    m_state_mahony.m_orientation(2) = stateBuffer(2);
    m_state_mahony.m_orientation(3) = stateBuffer(3);
    m_state_mahony.m_angular_velocity(0) = stateBuffer(4);
    m_state_mahony.m_angular_velocity(1) = stateBuffer(5);
    m_state_mahony.m_angular_velocity(2) = stateBuffer(6);
    m_state_mahony.m_gyroscope_bias(0) = stateBuffer(7);
    m_state_mahony.m_gyroscope_bias(1) = stateBuffer(8);
    m_state_mahony.m_gyroscope_bias(2) = stateBuffer(9);
    return true;
}

bool iDynTree::AttitudeMahonyFilter::setInternalStateInitialOrientation(const iDynTree::Span< double >& orientationBuffer)
{
    if ((size_t)orientationBuffer.size() != m_state_mahony.m_orientation.size())
    {
        iDynTree::reportError("AttitudeMahonyFilter", "setInternalStateInitialOrientation", "orientation size mismatch, using default state");
        return false;
    }
    m_state_mahony.m_orientation(0) = orientationBuffer(0);
    m_state_mahony.m_orientation(1) = orientationBuffer(1);
    m_state_mahony.m_orientation(2) = orientationBuffer(2);
    m_state_mahony.m_orientation(3) = orientationBuffer(3);

    return true;
}
