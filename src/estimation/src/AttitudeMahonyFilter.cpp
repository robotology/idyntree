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

iDynTree::Vector3 crossVector(const iDynTree::Vector3& a, const iDynTree::Vector3& b)
{
    iDynTree::Vector3 out;
    iDynTree::toEigen(out) = iDynTree::toEigen(a).cross(iDynTree::toEigen(b));
    return out;
}

iDynTree::Matrix3x3 mapR3Toso3(const iDynTree::Vector3& omega)
{
    iDynTree::Matrix3x3 out;    
    out(0, 0) = 0; out(0, 1) = -omega(2); out(0,2) = omega(1);
    out(1, 0) = omega(2); out(1, 1) = 0; out(1,2) = -omega(0);
    out(2, 0) = -omega(1); out(2, 1) = omega(0); out(2,2) = 0;
    return out;
}

bool checkSkewSymmetricity(const iDynTree::Matrix3x3& S)
{
    bool flag = false;
    if ( (iDynTree::toEigen(S) + iDynTree::toEigen(S).transpose()).isZero() )
    {
        flag = true;
    }
    return flag;
}

iDynTree::Vector3 mapso3ToR3(const iDynTree::Matrix3x3& S)
{    
    //TODO: need to check skey-symmetricity: S + transpose(S) = 0    
    iDynTree::Vector3 out;
    if (checkSkewSymmetricity(S))
    {
        out(0) = S(2, 1);
        out(1) = S(0, 2);
        out(2) = S(1, 0);
    }
    else
    {
        iDynTree::reportError("AttitudeMahonyFilter","mapso3ToR3","Passed matrix is not skew symmetric, returning zero vector");
        out.zero();
    }
    return out;
}

double innerProduct(const iDynTree::Vector3 a, const iDynTree::Vector3& b)
{
    return iDynTree::toEigen(a).dot(iDynTree::toEigen(b));
}

double realPartOfQuaternion(const iDynTree::Quaternion& q)
{   
    return q(0);
}

iDynTree::Vector3 imaginaryPartOfQuaternion(const iDynTree::Quaternion &q)
{
    iDynTree::Vector3 v;
    v(0) = q(1);
    v(1) = q(2);
    v(2) = q(3);
    return v;
}

iDynTree::Quaternion composeQuaternion(const iDynTree::Quaternion &q1, const iDynTree::Quaternion &q2)
{
    iDynTree::Quaternion out;
    double s1 = realPartOfQuaternion(q1);
    iDynTree::Vector3 v1 = imaginaryPartOfQuaternion(q1);
    
    double s2 = realPartOfQuaternion(q2);    
    iDynTree::Vector3 v2 = imaginaryPartOfQuaternion(q2);
    
    iDynTree::Vector3 imagOut;
    iDynTree::toEigen(imagOut) = iDynTree::toEigen(v2)*s1 + iDynTree::toEigen(v1)*s2 + iDynTree::toEigen(crossVector(v1, v2));
    out(0) = s1*s2 - innerProduct(v1, v2);
    out(1) = imagOut(0);
    out(2) = imagOut(1);
    out(3) = imagOut(2);    
    
    return out;
}

iDynTree::Quaternion pureQuaternion(const iDynTree::Vector3& bodyFixedFrameVelocityInInertialFrame)
{
    iDynTree::Quaternion p;
    p(0) = 0;
    p(1) = bodyFixedFrameVelocityInInertialFrame(0);
    p(2) = bodyFixedFrameVelocityInInertialFrame(1);
    p(3) = bodyFixedFrameVelocityInInertialFrame(2);    
    return p;
}

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

iDynTree::AttitudeMahonyFilter::AttitudeMahonyFilter()
{
    m_params.timeStepInSeconds = 0.001;
    m_params.kp = 10.0;
    m_params.ki = 10.0; 
    m_params.useMagnetoMeterMeasurements = false;
    m_params.confidenceMagnetometerMeasurements = 0.0;   
    
    m_state.m_orientation.zero();
    m_state.m_orientation(0) = 1.0;
    
    m_state.m_gyroscope_bias.zero();
    
    m_initial_state = m_state;
    
    m_gravity_direction.zero();
    m_gravity_direction(2) = 1.0;
    
    m_earth_magnetic_field_direction.zero();
    m_earth_magnetic_field_direction(2) = 1.0;
    
    m_orientationInRPY.zero();
    m_orientationInSO3.Identity();
    
    m_omega_mes.zero();
    m_Omega_y.zero();
}


void iDynTree::AttitudeMahonyFilter::setTimeStampInSeconds(double timestampInSeconds)
{
    m_params.timeStepInSeconds = timestampInSeconds;
}

void iDynTree::AttitudeMahonyFilter::setConfidenceForMagnetometerMeasurements(double confidence)
{
    if (m_params.useMagnetoMeterMeasurements == false)
    {
        iDynTree::reportWarning("AttitudeMahonyFilter", "setConfidenceForMagnetometerMeasurements", "not using magnetometer measurements, setting confidence to zero.");
        m_params.confidenceMagnetometerMeasurements = 0.0;
    }
    m_params.confidenceMagnetometerMeasurements = confidence;
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
    m_params.useMagnetoMeterMeasurements = flag;
    if (flag == false)
    {
        iDynTree::reportWarning("AttitudeMahonyFilter", "useMagnetoMeterMeasurements", "setting confidence on magnetometer measurements to zero.");
        m_params.confidenceMagnetometerMeasurements = 0.0;
    }
}

bool iDynTree::AttitudeMahonyFilter::getInternalInitialState(iDynTree::Span< double >& stateBuffer) const
{
    stateBuffer(0) = m_initial_state.m_orientation(0);
    stateBuffer(1) = m_initial_state.m_orientation(1);
    stateBuffer(2) = m_initial_state.m_orientation(2);
    stateBuffer(3) = m_initial_state.m_orientation(3);
    stateBuffer(4) = m_initial_state.m_gyroscope_bias(0);
    stateBuffer(5) = m_initial_state.m_gyroscope_bias(1);
    stateBuffer(6) = m_initial_state.m_gyroscope_bias(2);
    return true;
}

bool iDynTree::AttitudeMahonyFilter::getInternalState(iDynTree::Span< double >& stateBuffer) const
{
    stateBuffer(0) = m_state.m_orientation(0);
    stateBuffer(1) = m_state.m_orientation(1);
    stateBuffer(2) = m_state.m_orientation(2);
    stateBuffer(3) = m_state.m_orientation(3);
    stateBuffer(4) = m_state.m_gyroscope_bias(0);
    stateBuffer(5) = m_state.m_gyroscope_bias(1);
    stateBuffer(6) = m_state.m_gyroscope_bias(2);
    return true;
}

size_t iDynTree::AttitudeMahonyFilter::getInternalStateSize() const
{
    double size = 0;
    size += m_state.m_orientation.size();
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
    if (stateBuffer.size() != 0)
    {
        iDynTree::reportError("AttitudeMahonyFilter", "setInternalState", "state size mismatch, using default state");
        return false;
    }
    m_state.m_orientation(0) = stateBuffer(0);
    m_state.m_orientation(1) = stateBuffer(1);
    m_state.m_orientation(2) = stateBuffer(2);
    m_state.m_orientation(3) = stateBuffer(3);
    m_state.m_gyroscope_bias(0) = stateBuffer(4);
    m_state.m_gyroscope_bias(1) = stateBuffer(5);
    m_state.m_gyroscope_bias(2) = stateBuffer(6);
    return true;
}

bool iDynTree::AttitudeMahonyFilter::setParameters(const iDynTree::AttitudeMahonyFilterParameters& params)
{   m_params = params;        
    return true;
}

bool iDynTree::AttitudeMahonyFilter::updateFilterWithMeasurements(const iDynTree::LinearAccelerometerMeasurements& linAccMeas, const iDynTree::GyroscopeMeasurements& gyroMeas)
{   
    iDynTree::Matrix3x3 S_acc = getAngVelSkewSymmetricMatrixFromMeasurements(linAccMeas, m_gravity_direction.reverse(), 1-m_params.confidenceMagnetometerMeasurements, m_orientationInSO3);
    iDynTree::toEigen(m_omega_mes) = -iDynTree::toEigen(mapso3ToR3(S_acc));                
    m_Omega_y = gyroMeas;
    return true;
}

bool iDynTree::AttitudeMahonyFilter::updateFilterWithMeasurements(const iDynTree::LinearAccelerometerMeasurements& linAccMeas, const iDynTree::GyroscopeMeasurements& gyroMeas, const iDynTree::MagnetometerMeasurements& magMeas)
{
    if (m_params.useMagnetoMeterMeasurements == false)
    {
        iDynTree::reportWarning("AttitudeMahonyFilter", "updateFilterWithMeasurements", "useMagnetoMeterMeasurements set to false, using only accelerometer measurements");
        return updateFilterWithMeasurements(linAccMeas, gyroMeas);
    }
    
    iDynTree::Matrix3x3 S_acc = getAngVelSkewSymmetricMatrixFromMeasurements(linAccMeas, m_gravity_direction.reverse(), 1-m_params.confidenceMagnetometerMeasurements, m_orientationInSO3);      
    iDynTree::Matrix3x3 S_mag = getAngVelSkewSymmetricMatrixFromMeasurements(magMeas, m_earth_magnetic_field_direction, m_params.confidenceMagnetometerMeasurements, m_orientationInSO3);
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
    iDynTree::toEigen(m_state.m_orientation) = iDynTree::toEigen(m_state.m_orientation) + iDynTree::toEigen((composeQuaternion(m_state.m_orientation, correction)))*(m_params.timeStepInSeconds*0.5);
    iDynTree::toEigen(m_state.m_gyroscope_bias) = iDynTree::toEigen(m_state.m_gyroscope_bias) - (iDynTree::toEigen(m_omega_mes)*m_params.ki);
    
    m_orientationInSO3 = iDynTree::Rotation::RotationFromQuaternion(m_state.m_orientation);
    m_orientationInRPY = iDynTree::Rotation::RotationFromQuaternion(m_state.m_orientation).asRPY();
    return true;
}

