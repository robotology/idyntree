/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 : 
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef ATTITUDE_MAHONY_FILTER_H
#define ATTITUDE_MAHONY_FILTER_H

#include <iDynTree/Estimation/AttitudeEstimator.h>
#include <iDynTree/Core/Direction.h>
#include <iDynTree/Core/EigenHelpers.h>

namespace iDynTree 
{
    
struct AttitudeMahonyFilterParameters {
    //TODO: add default values and docs
    double timeStepInSeconds{0.001};
    double kp{10.0};
    double ki{10.0}; 
    bool useMagnetoMeterMeasurements{false};
    double confidenceMagnetometerMeasurements{0.0};    
    
};

/**
 * @class AttitudeMahonyFilter Implements an explicit passive complementary filter on quaternion groups
 */
class AttitudeMahonyFilter : public IAttitudeEstimator
{
public:
    AttitudeMahonyFilter();
    void useMagnetoMeterMeasurements(bool flag);
    void setConfidenceForMagnetometerMeasurements(double confidence);
    void setGainkp(double kp);
    void setGainki(double ki);
    void setTimeStampInSeconds(double timestampInSeconds);
    
    
    // TODO: specify if this resets also the internal state
    bool setParameters(const AttitudeMahonyFilterParameters& params);
    void getParameters(AttitudeMahonyFilterParameters& params);
    
    bool updateFilterWithMeasurements(const iDynTree::LinearAccelerometerMeasurements& linAccMeas, 
                                    const iDynTree::GyroscopeMeasurements& gyroMeas) override;  


    bool updateFilterWithMeasurements(const iDynTree::LinearAccelerometerMeasurements& linAccMeas, 
                                            const iDynTree::GyroscopeMeasurements& gyroMeas, 
                                            const iDynTree::MagnetometerMeasurements& magMeas) override; 

    bool propagateStates() override;
                                            
    bool getOrientationEstimateAsRotationMatrix(iDynTree::Rotation& rot) override;

    bool getOrientationEstimateAsQuaternion(iDynTree::Quaternion& q) override;

    bool getOrientationEstimateAsRPY(iDynTree::RPY& rpy) override;   

    // State 
    size_t getInternalStateSize() const override;  
    bool getInternalState(iDynTree::Span<double> & stateBuffer) const override;
    bool getInternalInitialState(iDynTree::Span<double> & stateBuffer) const override;
    bool setInternalState(iDynTree::Span<double> & stateBuffer) override;
private:
    AttitudeMahonyFilterParameters m_params;
    struct {
        iDynTree::Quaternion m_orientation; 
        iDynTree::Vector3 m_gyroscope_bias; 
    } m_state, m_initial_state;
    
    iDynTree::Rotation m_orientationInSO3;
    iDynTree::RPY m_orientationInRPY;
    
    iDynTree::Vector3 m_omega_mes; // notation from the paper Non linear complementary filters on the special orthogonal group
    iDynTree::GyroscopeMeasurements m_Omega_y; // notation from the paper Non linear complementary filters on the special orthogonal group
    
    iDynTree::Direction m_gravity_direction; // default set to e3
    iDynTree::Direction m_earth_magnetic_field_direction; // denoted by m_A
};

}

#endif
