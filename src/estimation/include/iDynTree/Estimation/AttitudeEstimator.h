/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_ATTITUDEESTIMATOR_H
#define IDYNTREE_ATTITUDEESTIMATOR_H

#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/Rotation.h>

#include <iostream>

namespace iDynTree
{
    typedef iDynTree::Vector3 LinearAccelerometerMeasurements;
    typedef iDynTree::Vector3 GyroscopeMeasurements;
    typedef iDynTree::Vector3 MagnetometerMeasurements;

    typedef iDynTree::Vector4 UnitQuaternion;
    typedef iDynTree::Vector3 RPY;

    /** @struct state internal state of the estimator
         * @var state::m_orientation
         * orientation estimate in \f$ \mathbb{R}^4 \f$ quaternion representation
         * @var state::m_orientation
         * angular velocity estimate in \f$ \mathbb{R}^3 \f$
         * @var state::m_orientation
         * gyroscope bias estimate in \f$ \mathbb{R}^3 \f$
         */
        struct AttitudeEstimatorState
        {
            iDynTree::UnitQuaternion m_orientation;
            iDynTree::Vector3 m_angular_velocity;
            iDynTree::Vector3 m_gyroscope_bias;
        };

    /**
     * @class IAttitudeEstimator generic interface for attitude estimator classes
     *
     * The aim is to implement different attitude estimators as a block that takes IMU measurements
     * as inputs and gives attitude estimates as outputs. This way the underlying implementation is abstracted
     * and the user only has to set a few parameters and run the estimator.
     *
     * The general procedure to use the estimators would be,
     * - instantiate the filter,
     * - set initial internal state
     * - in a loop,
     *     - update the filter with measurements
     *     - propagate the states
     *
     * However, additional methods to set and get parameters for the filter might be available with respect to the filters.
     *
     * The internal state of the estimator is described as \f$ X = \begin{bmatrix} {^A}q_B \\ {^B}\Omega_{A,B} \\ {^B}b \end{bmatrix}^T \f$
     * \f$ {^A}q_B \in \mathbb{R}^4 \f$  is the quaternion representing the orientation of a body(IMU) frame with respect to an inertial frame ,
     * \f$ {^B}\Omega_{A,B} \in \mathbb{R}^3 \f$  is the angular velocity of a body(IMU) frame with respect to an inertial frame, expressed in the body frame and
     * \f$ {^B}b \in \mathbb{R}^3 \f$  is the gyroscope bias expressed in the body frame.
     *
     */
    class IAttitudeEstimator
    {
    public:
        virtual ~IAttitudeEstimator();

        /**
         * @brief Update the filter with accelerometer and gyroscope measurements
         *
         * @param[in] linAccMeas proper (body acceleration - gravity) classical acceleration of the origin of the body frame B expressed in frame B
         * @param[in] gyroMeas angular velocity of body frame B with respect to an inertial fram A, expressed in frame B
         *
         * @note consider the current behavior of our system does not use magnetometer measurements and is calling this method to update measurements.
         *       Then, if we decide to turn the flag use_magnetometer_measurements to true, this will not guarantee that the magnetometer measurements
         *       will be used. The magnetometer measurements will be used only if we replace this function call with the other overlaoded function considering
         *       the magnetometer measurements.
         *
         * @return true/false if successful/not
         */
        virtual bool updateFilterWithMeasurements(const iDynTree::LinearAccelerometerMeasurements& linAccMeas,
                                                  const iDynTree::GyroscopeMeasurements& gyroMeas) = 0;

        /**
         * @brief Update the filter with accelerometer, gyroscope and magnetometer measurements
         *
         * @param[in] linAccMeas proper (body acceleration - gravity) classical acceleration of the origin of the body frame B expressed in frame B
         * @param[in] gyroMeas angular velocity of body frame B with respect to an inertial fram A, expressed in frame B
         * @param[in] magMeas magnetometer measurements expressed in frame B
         *
         * @return true/false if successful/not
         */
        virtual bool updateFilterWithMeasurements(const iDynTree::LinearAccelerometerMeasurements& linAccMeas,
                                                  const iDynTree::GyroscopeMeasurements& gyroMeas,
                                                  const iDynTree::MagnetometerMeasurements& magMeas) = 0;

        /**
         * @brief Propagate the states and associated uncertainties through properly defined propagation functions
         * The underlying implementation depends on the type of filter being implemented.
         *
         * @return true/false if successful/not
         */
        virtual bool propagateStates() = 0;

        /**
         * @brief Get orientation of the body with respect to inertial frame, in rotation matrix form
         *        If we denote \f$ A \f$ as inertial frame and \f$ B \f$ as the frame attached to the body,
         *        then this method gives us \f$ {^A}R_B \f$ as the rotation matrix
         * @param[out] rot Rotation matrix
         * @return true/false if successful/not
         */
        virtual bool getOrientationEstimateAsRotationMatrix(iDynTree::Rotation& rot) = 0;

        /**
         * @brief Get orientation of the body with respect to inertial frame, in unit quaternion form
         *        If we denote \f$ A \f$ as inertial frame and \f$ B \f$ as the frame attached to the body,
         *        then this method gives us \f$ {^A}q_B as the quaternion \f$
         *
         * @note quaternion has the form (real, imaginary) and is normalized
         * @note Usually a rotation matrix can be described using two quaternions due to its double-connectedness property
         *       Depending on the specific filter, the initial state and the trajectory of the system, we could obtain
         *       one quaternion or the other(opposite spin), depending on the system dynamics.
         *
         * @param[out] q UnitQuaternion
         * @return true/false if successful/not
         */
        virtual bool getOrientationEstimateAsQuaternion(iDynTree::UnitQuaternion& q) = 0;

        /**
         * @brief Get orientation of the body with respect to inertial frame, in Euler's RPY form
         *        If we denote \f$ A \f$ as inertial frame and \f$ B \f$ as the frame attached to the body,
         *        then this method gives us the RPY 3d vector of Euler Angles when composed together gives us \f$ {^A}R_B \f$ as the rotation matrix
         *        where \f$ {^A}R_B = Rot_z(yaw)Rot_y(pitch)Rot_x(roll)\f$.
         *        For more details about the range of the RPY Euler angles, please refer the documentation of
         *   <a href="https://github.com/robotology/idyntree/blob/c8bf721b771fa4b1e7c3a940632e121060719a19/src/core/include/iDynTree/Core/Rotation.h#L167">GetRPY()</a>
         *
         * @param[out] rpy 3D vector containing roll pitch yaw angles
         * @return true/false if successful/not
         */
        virtual bool getOrientationEstimateAsRPY(iDynTree::RPY& rpy) = 0;

        /**
         * @brief Get dimension of the state vector
         * @return size_t size of state vector
         */
        virtual size_t getInternalStateSize() const = 0;

        /**
         * @brief Get internal state of the estimator
         *        The internal state of the estimator is described as \f$ X = \begin{bmatrix} {^A}q_B \\ {^B}\Omega_{A,B} \\ {^B}b \end{bmatrix}^T \f$
         *        \f$ {^A}q_B \in \mathbb{R}^4 \f$  is the quaternion representing the orientation of a body(IMU) frame with respect to an inertial frame ,
         *        \f$ {^B}\Omega_{A,B} \in \mathbb{R}^3 \f$  is the angular velocity of a body(IMU) frame with respect to an inertial frame, expressed in the body frame and
         *        \f$ {^B}b \in \mathbb{R}^3 \f$  is the gyroscope bias expressed in the body frame.
         *        The default internal state of the estimator would be \f$ X = \begin{bmatrix} 1.0 \\ 0_{1 \times 3} \\ 0_{1 \times 3} \\ 0_{1 \times 3} \end{bmatrix}^T \f$
         * @param[out] stateBuffer Span object as reference of the container where state vector should be copied to
         * @return true/false if successful/not
         */
        virtual bool getInternalState(const iDynTree::Span<double> & stateBuffer) const = 0;

        /**
         * @brief Get initial internal state of the estimator
         *        The internal state of the estimator is described as \f$ X = \begin{bmatrix} {^A}q_B \\ {^B}\Omega_{A,B} \\ {^B}b \end{bmatrix}^T \f$
         *        \f$ {^A}q_B \in \mathbb{R}^4 \f$  is the quaternion representing the orientation of a body(IMU) frame with respect to an inertial frame ,
         *        \f$ {^B}\Omega_{A,B} \in \mathbb{R}^3 \f$  is the angular velocity of a body(IMU) frame with respect to an inertial frame, expressed in the body frame and
         *        \f$ {^B}b \in \mathbb{R}^3 \f$  is the gyroscope bias expressed in the body frame.
         *        The default internal state of the estimator would be \f$ X = \begin{bmatrix} 1.0 \\ 0_{1 \times 3} \\ 0_{1 \times 3} \\ 0_{1 \times 3} \end{bmatrix}^T \f$
         * @param[out] stateBuffer Span object as reference of the container where state vector should be copied to
         * @return true/false if successful/not
         */
        virtual bool getDefaultInternalInitialState(const iDynTree::Span<double> & stateBuffer) const = 0;

        /**
         * @brief set internal state of the estimator.
         *        The internal state of the estimator is described as \f$ X = \begin{bmatrix} {^A}q_B \\ {^B}\Omega_{A,B} \\ {^B}b \end{bmatrix}^T \f$
         *        \f$ {^A}q_B \in \mathbb{R}^4 \f$  is the quaternion representing the orientation of a body(IMU) frame with respect to an inertial frame ,
         *        \f$ {^B}\Omega_{A,B} \in \mathbb{R}^3 \f$  is the angular velocity of a body(IMU) frame with respect to an inertial frame, expressed in the body frame and
         *        \f$ {^B}b \in \mathbb{R}^3 \f$  is the gyroscope bias expressed in the body frame.
         * @param[in] stateBuffer Span object as reference of the container from which the internal state vector should be assigned. The size of the buffer should be 10.
         * @return true/false if successful/not
         */
        virtual bool setInternalState(const iDynTree::Span<double> & stateBuffer) = 0;

        /**
         * @brief set the initial orientation for the internal state of the estimator.
         *        The initial orientation for the internal state of the estimator is described as \f$ {^A}q_B \f$
         *        \f$ {^A}q_B \in \mathbb{R}^4 \f$  is the quaternion representing the orientation of a body(IMU) frame with respect to an inertial frame ,
         * @param[in] stateBuffer Span object as reference of the container from which the inital orientaiton for internal state vector should be assigned. The size of the buffer should be 4.
         * @return true/false if successful/not
         */
        virtual bool setInternalStateInitialOrientation(const iDynTree::Span<double>& orientationBuffer) = 0;
    };

}
#endif
