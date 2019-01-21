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

    typedef iDynTree::Vector4 Quaternion;
    typedef iDynTree::Vector3 RPY;

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
     */
    class IAttitudeEstimator
    {
    public:
        virtual ~IAttitudeEstimator();

        /**
         * @brief Update the filter with accelerometer and gyroscope measurements
         *
         * @param[in] linAccMeas left trivialized 3D vector of linear proper sensor acceleration measuements
         * @param[in] gyroMeas left trivialized 3D vector of angular velocity measurements
         *
         * @note left trivialized angular velocity means the angular velocity of body frame B with respect to an inertial fram A, expressed in frame B
         * @return true/false if successful/not
         */
        virtual bool updateFilterWithMeasurements(const iDynTree::LinearAccelerometerMeasurements& linAccMeas,
                                          const iDynTree::GyroscopeMeasurements& gyroMeas) = 0;

        /**
         * @brief Update the filter with accelerometer, gyroscope and magnetometer measurements
         *
         * @param[in] linAccMeas left trivialized 3D vector of linear proper sensor acceleration measuements
         * @param[in] gyroMeas left trivialized 3D vector of angular velocity measurements
         * @param[in] magMeas left trivialized 3D vector of magnetometer measurements
         *
         * @note left trivialized angular velocity means the angular velocity of body frame B with respect to an inertial fram A, expressed in frame B
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
         *
         * @param[out] rot Rotation matrix
         * @return true/false if successful/not
         */
        virtual bool getOrientationEstimateAsRotationMatrix(iDynTree::Rotation& rot) = 0;

        /**
         * @brief Get orientation of the body with respect to inertial frame, in quaternion form
         *
         * @note quaternion has the form (real, imaginary) and is normalized
         *
         * @param[out] q Quaternion
         * @return true/false if successful/not
         */
        virtual bool getOrientationEstimateAsQuaternion(iDynTree::Quaternion& q) = 0;

        /**
         * @brief Get orientation of the body with respect to inertial frame, in Euler's RPY form
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
         *
         * @param[out] stateBuffer Span object as reference of the container where state vector should be copied to
         * @return true/false if successful/not
         */
        virtual bool getInternalState(iDynTree::Span<double> & stateBuffer) const = 0;

        /**
         * @brief Get initial internal state of the estimator
         *
         * @param[out] stateBuffer Span object as reference of the container where state vector should be copied to
         * @return true/false if successful/not
         */
        virtual bool getInternalInitialState(iDynTree::Span<double> & stateBuffer) const = 0;

        /**
         * @brief set internal state of the estimator
         *
         * @param[in] stateBuffer Span object as reference of the container from which the internal state vector should be assigned
         * @return true/false if successful/not
         */
        virtual bool setInternalState(iDynTree::Span<double> & stateBuffer) = 0;
    };

}
#endif
