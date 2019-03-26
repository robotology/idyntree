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

namespace iDynTree
{

/**
* @struct AttitudeMahonyFilterParameters Parameters to set up the quaternion EKF
* @var AttitudeMahonyFilterParameters::time_step_in_seconds
* discretization time step in seconds, default value: \f$ 0.001 s \f$
* @var AttitudeMahonyFilterParameters::kp
* Mahony \f$ K_p \f$ gain over the correction from IMU measurements, default value: \f$ 1.0 \f$
* @var AttitudeMahonyFilterParameters::ki
* Mahony \f$ K_i \f$ gain over the gyro bias evolution, default value: \f$ 1.0 \f$
* @var AttitudeMahonyFilterParameters::use_magenetometer_measurements
* flag to enable the use of magnetometer measurement for yaw correction, default value: false
* @var AttitudeMahonyFilterParameters::confidence_magnetometer_measurements
* confidence on magnetometer measurements, default value: \f$ 0.0 \f$
*/
struct AttitudeMahonyFilterParameters {
    double time_step_in_seconds{0.001};
    double kp{1.0};
    double ki{1.0};
    bool use_magnetometer_measurements{false};
    double confidence_magnetometer_measurements{0.0};
};


/**
 * @class AttitudeMahonyFilter Implements an explicit passive complementary filter on quaternion groups
 * described in the paper <a href="https://hal.archives-ouvertes.fr/hal-00488376/document">Non-linear complementary filters on SO3 groups</a>
 *
 * The filter is used to estimate the states \f$ X = \begin{bmatrix} {^A}q_B \\ {^B}\Omega_{A,B} \\ {^B}b \end{bmatrix}^T \f$
 * where \f$ {^A}q_B \in \mathbb{R}^4 \f$  is the quaternion representing the orientation of a body(IMU) frame with respect to an inertial frame ,
 *       \f$ {^B}\Omega_{A,B} \in \mathbb{R}^3 \f$  is the angular velocity of a body(IMU) frame with respect to an inertial frame, expressed in the body frame and
 *       \f$ {^B}b \in \mathbb{R}^3 \f$  is the gyroscope bias expressed in the body frame.
 * @note: we will drop the subscripts and superscripts in the rest of the documentation for convenience
 *
 * The discretized dynamics of the filter is implemented in the propagateStates() method and is described by the following equations,
 * \f$ q_{k+1} = q_{k} + \Delta t \frac{1}{2}q_{k} \circ \begin{bmatrix} 0 \\ \Omega_y_{k+1} - b_k + K_p \omega_{mes_{k+1}}\end{bmatrix}\f$
 * \f$ \Omega_{k+1} = \Omega_y_{k+1} - b_k \f$
 * \f$ b_{k+1} = b_k - K_i \Delta t \frac{1}{2} \omega_{mes_{k+1}} \f$
 *
 * The updateFilterWithMeasurements() uses the recent IMU measurements to compute the term \f$ \omega_{mes} \f$ which gives the vectorial from accelerometer and magnetometer measurements
 * \f$ \omega_{mes} = -(\Sigma{n}{i=1} \frac{k_i}{2} (v_i \hat{v}_i^T - \hat{v}_i v_i^T) )^{\vee} \f$
 * where \f$ v_i \f$ is the normalized accelerometer or magnetometer measurement,
 * \f$ \hat{v_i} \f$ is the vector obtained from the orientation estimated combined with gravity direction or absolute magnetic field direction, for e.g, \f$ \hat{v_acc} = {^w}R_b^T e_3 \f$
 * and \f$ k_i \f$ is the confidence weight on the i-th measurement. In our case, i = 1 or 2.
 *
 * The usage of the attitude estimator can be as follows,
 * - After instantiation, the parameters of the filter can be set using the individual parameter methods or the struct method.
 * - The filter state can be initialized by calling the setInternalState() method
 * - Once initialized, the following filter methods can be run in a loop to get the orientation estimates,
 *     - updateFilterWithMeasurements() method to pass the recent measurements to the filter
 *     - propagateStates() method to propagate the states through the system dynamics and correcting using the updated measurements
 *     - getInternalState() or getOrientationEstimate*() methods to get the entire state estimate or only the attitude estimated in desired representation
 */
class AttitudeMahonyFilter : public IAttitudeEstimator
{
public:
    AttitudeMahonyFilter();

    /**
     * @brief set flag to use magnetometer measurements
     * @param[in] flag enable/disable magnetometer measurements
     */
    void useMagnetoMeterMeasurements(bool flag);

    /**
     * @brief set the confidence weights on magenetometer measurements, if used
     * @param[in] confidence can take values between \f$ [0, 1] \f$
     */
    void setConfidenceForMagnetometerMeasurements(double confidence);

    /**
     * @brief set the Kp gain
     * @param[in] kp gain
     */
    void setGainkp(double kp);

    /**
     * @brief set the Ki gain
     * @param[in] ki gain
     */
    void setGainki(double ki);

    /**
     * @brief set discretization time step in seconds
     * @param[in] timestepInSeconds time step
     */
    void setTimeStepInSeconds(double timestepInSeconds);

    /**
     * @brief Set the gravity direction assumed by the filter (for computing orientation vectorial from accelerometer)
     * @param[in] gravity_dir gravity direction
     */
    void setGravityDirection(const iDynTree::Direction& gravity_dir);

    /**
     * @brief Set filter parameters with the struct members.
     * This does not reset the internal state.
     * @param[in] params object of AttitudeMahonyFilterParameters passed as a const reference
     * @return true/false if successful/not
     */
    bool setParameters(const AttitudeMahonyFilterParameters& params)
    {
        m_params_mahony = params;
        return true;
    }

    /**
     * @brief Get filter parameters as a struct.
     * @param[out] params object of AttitudeMahonyFilterParameters passed as reference
     */
    void getParameters(AttitudeMahonyFilterParameters& params) {params = m_params_mahony;}

    bool updateFilterWithMeasurements(const iDynTree::LinearAccelerometerMeasurements& linAccMeas,
                                      const iDynTree::GyroscopeMeasurements& gyroMeas) override;
    bool updateFilterWithMeasurements(const iDynTree::LinearAccelerometerMeasurements& linAccMeas,
                                      const iDynTree::GyroscopeMeasurements& gyroMeas,
                                      const iDynTree::MagnetometerMeasurements& magMeas) override;
    bool propagateStates() override;
    bool getOrientationEstimateAsRotationMatrix(iDynTree::Rotation& rot) override;
    bool getOrientationEstimateAsQuaternion(iDynTree::UnitQuaternion& q) override;
    bool getOrientationEstimateAsRPY(iDynTree::RPY& rpy) override;
    size_t getInternalStateSize() const override;
    bool getInternalState(const iDynTree::Span<double> & stateBuffer) const override;
    bool getDefaultInternalInitialState(const iDynTree::Span<double> & stateBuffer) const override;
    bool setInternalState(const iDynTree::Span<double> & stateBuffer) override;
    bool setInternalStateInitialOrientation(const iDynTree::Span<double>& orientationBuffer) override;

protected:
    AttitudeMahonyFilterParameters m_params_mahony;              ///< struct holding the Mahony filter parameters
    AttitudeEstimatorState m_state_mahony, m_initial_state_mahony;
private:
    iDynTree::Rotation m_orientationInSO3;                ///< orientation estimate as rotation matrix \f$ {^A}R_B \f$ where \f$ A \f$ is inertial frame and \f$ B \f$ is the frame attached to the body
    iDynTree::RPY m_orientationInRPY;                     ///< orientation estimate as a 3D vector in RPY representation, where \f$ {^A}R_B = Rot_z(yaw)Rot_y(pitch)Rot_x(roll) \f$

    iDynTree::Vector3 m_omega_mes;                        ///< vectorial estimate from accelerometer and magnetometer measurements, \f$ \omega_{mes} \in \mathbb{R}^3 \f$, notation from the paper Non linear complementary filters on the special orthogonal group
    iDynTree::GyroscopeMeasurements m_Omega_y;            ///< gyroscope measurement, \f$ \Omega_{y} \in \mathbb{R}^3 \f$, notation from the paper Non linear complementary filters on the special orthogonal group

    iDynTree::Direction m_gravity_direction;              ///< direction of the gravity vector expressed in the inertial frame denoted by \f$ A \f$, default set to \f$ e_3 = \begin{bmatrix} 0 & 0 & 1.0 \end{bmatrix}^T \f$
    iDynTree::Direction m_earth_magnetic_field_direction; ///< direction of absolute magnetic field expressed in the inertial frame denoted by \f$ A \f$, default set to \f$ {^A}m = \begin{bmatrix} 0 & 0 & 1.0 \end{bmatrix}^T \f$
};

}

#endif
