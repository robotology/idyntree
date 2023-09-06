// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <iDynTree/VectorDynSize.h>
#include <iDynTree/MatrixDynSize.h>
#include <iDynTree/Utils.h>
#include <iDynTree/EigenHelpers.h>
#include <vector>

namespace iDynTree
{
    /**
     * @class DiscreteKalmanFilterHelper Time Invariant Discrete Kalman Filter with additive Gaussian noise
     *
     * The Kalman filter can be constructed by giving the system design matrices A, B, C and D.
     * where A is the state transition matrix, B is the control input matrix,
     * C is the output matrix and D the feed through matrix.
     * These matrices can be used to describe a stochastic model for a linear dynamical system.
     *
     * \f[ x_{k+1} = A x_{k} + B u_{k} + w_k \f]
     * \f[ y_{k+1} = C x_{k+1} + D u_{k} + v_k \f]
     *
     * Once the filter is constructed, the system noise and measurement noise covariance matrices can be set.
     * The filter can be run, after setting initial state and state covariance matrix.
     * The filter init method is called to check if the filter is properly configured and is ready to use.
     *
     * @warning care must be taken to design these matrices with proper dimensions, so that the filter does not crash
     *
     * Once the filter is configured, the filter algorithm can be run in a loop by,
     * - setting input vector
     * - prediction step which propagates the state through the modeled state dynamics
     * - setting the measurement vector
     * - correct the predicted states with the incoming measurements
     *
     */
    class DiscreteKalmanFilterHelper
    {
    public:
        DiscreteKalmanFilterHelper();

        /**
         * @brief Describes the state propagation for a given dynamical system
         *       and the measurement model given the available measurements.
         *        If state of the system is denoted by \f$ x \f$, the control input by \f$ u \f$
         *        and the measurements by \f$ y \f$, then
         *        the system dynamics is given as \f$ x_{k+1} = A x_{k} + B u_{k} + w_k \f$
         *        and the measurement model is given by \f$ y_{k+1} = C x_{k+1} + D u_{k} + v_k \f$
         *
         * @note all matrices are assumed to be time-invariant
         * @param[in] A state transition matrix
         * @param[in] B control input matrix mapping inputs to states
         * @param[in] C output matrix mapping states to measurements
         * @param[in] D feed through matrix mapping inputs to measurements
         * @return bool true/false if filter was constructed successfully or not
         */
        bool constructKalmanFilter(const iDynTree::MatrixDynSize& A,
                                const iDynTree::MatrixDynSize& B,
                                const iDynTree::MatrixDynSize& C,
                                const iDynTree::MatrixDynSize& D);
        /**
         * @overload
         */
        bool constructKalmanFilter(const iDynTree::MatrixDynSize& A,
                                const iDynTree::MatrixDynSize& B,
                                const iDynTree::MatrixDynSize& C);
        /**
         * @overload
         */
        bool constructKalmanFilter(const iDynTree::MatrixDynSize& A,
                                const iDynTree::MatrixDynSize& C);

        /**
         * @brief Set initial state of the Kalman filter
         * @warning this method must be called before calling kfInit()
         * @param[in] x0 initial state of dimension \f$ dim_x \times 1 \f$
         * @return bool true/false successful or not
         */
        bool kfSetInitialState(const iDynTree::VectorDynSize& x0);

        /**
         * @brief Sets the state covariance matrix
         * @warning this method must be called before calling kfInit()
         * @param[in] P state covariance matrix of dimensions \f$ dim_x \times dim_x \f$
         * @return bool true/false successful or not
         */
        bool kfSetStateCovariance(const iDynTree::MatrixDynSize& P);

        /**
         * @brief Sets the system noise covariance matrix
         * @warning this method must be called before calling kfInit()
         * @param[in] Q system noise covariance matrix of dimensions \f$ dim_x \times dim_x \f$
         * @return bool true/false successful or not
         */
        bool kfSetSystemNoiseCovariance(const iDynTree::MatrixDynSize& Q);

        /**
         * @brief Sets the measurement noise covariance matrix
         * @warning this method must be called before calling kfInit()
         * @param[in] R measurement covariance matrix of dimensions \f$ dim_y \times dim_y \f$
         * @return bool true/false successful or not
         */
        bool kfSetMeasurementNoiseCovariance(const iDynTree::MatrixDynSize& R);

        /**
         * @brief This method checks if the filter is properly constructed and configured
         * i.e. if initial states and covariance matrices are set.
         * @warning this method must be called before calling kfPredict() or kfUpdate()
         * @return bool true/false successful or not
         */
        bool kfInit();

        /**
         * @brief Set inputs for the Kalman filter
         *
         * @param[in] u input vector of dimension \f$ dim_u \times 1 \f$
         * @return bool true/false successful or not
         */
        bool kfSetInputVector(const iDynTree::VectorDynSize& u);

        /**
         * @brief Runs one step of the Discrete Kalman Filter prediction equation
         *        described by
         *        \f$ \hat{x}_{k+1} = A x_{k} + B u_{k} \f$
         *        \f$ \hat{P}_{k+1} = A_k P_k A_k^T + Q \f$
         *
         * @warning this function can be called only after setting up the filter properly through kfInit() step
         * @note in case if B matrix is constructed, this function should be once called every step,
         *       after setting up the input vector using kfSetInputVector() method.
         *
         * @return bool true/false if successful or not
         */
        bool kfPredict();

        /**
         * @brief Set measurements for the Kalman filter
         *
         * @param[in] y input vector of dimension \f$ dim_y \times 1 \f$
         * @return bool true/false successful or not
         */
        bool kfSetMeasurementVector(const iDynTree::VectorDynSize& y);

        /**
         * @brief Runs one step of the Discrete Kalman Filter update equation
         *        described by
         *        \f$ \tilde{y}_{k+1} = C \hat{x}_{k+1} + D u_{k} \f$
         *        \f$ x_{k+1} = \hat{x}_{k+1} + K_{k+1}(\tilde{y}_{k+1} - z_{k+1}) \f$
         *        \f$ P_{k+1} = (I - K_{k+1} C) \hat{P}_{k+1} \f$
         * where K is the Kalman gain.
         * @warning this function can be called only after setting up the filter properly through kfInit() step
         * @note this function should be once called every step,
         *       after setting up the measurements vector using kfSetMeasurementVector() method.
         *
         * @return bool true/false if successful or not
         */
        bool kfUpdate();

        /**
         * @brief Get system state
         *
         * @param[out] x system state of dimension \f$ dim_x \times 1 \f$
         * @return bool true/false successful or not
         */
        bool kfGetStates(iDynTree::VectorDynSize &x);

        /**
         * @brief Get system state covariance
         *
         * @param[out] P system state covariance matrix of dimension \f$ dim_x \times dim_x \f$
         * @return bool true/false successful or not
         */
        bool kfGetStateCovariance(iDynTree::MatrixDynSize &P);

        /**
         * @brief Reset Kalman filter
         * Resets the Kalman filter and initializes with the internally stored states and
         * matrices initially set by the user.
         */
        bool kfReset();

        /**
         * @brief Reset Kalman filter with the given arguments
         * @warning the system matrices A, B, C and D are unchanged with the reset filter
         *
         * @param[in] x0 initial state vector of dimensions \f$ dim_x \times 1 \f$
         * @param[in] P state covariance matrix of dimensions \f$ dim_x \times dim_x \f$
         * @param[in] Q system noise covariance matrix of dimensions \f$ dim_x \times dim_x \f$
         * @param[in] R measurement noise covariance matrix of dimensions \f$ dim_y \times dim_y \f$
         */
        bool kfReset(const iDynTree::VectorDynSize& x0, const iDynTree::MatrixDynSize& P0,
                     const iDynTree::MatrixDynSize& Q, const iDynTree::MatrixDynSize& R);

    private:
        size_t m_dim_X;                                ///< state dimension
        size_t m_dim_Y;                                ///< output dimenstion
        size_t m_dim_U;                                ///< input dimension

        iDynTree::VectorDynSize m_x;                   ///< state at time instant k
        iDynTree::VectorDynSize m_x0;                  ///< buffer to store initial state
        iDynTree::VectorDynSize m_u;                   ///< input at time instant k
        iDynTree::VectorDynSize m_y;                   ///< measurements at time instant k

        iDynTree::MatrixDynSize m_A;                   ///< state transition matrix
        iDynTree::MatrixDynSize m_B;                   ///< control input matrix
        iDynTree::MatrixDynSize m_C;                   ///< output matrix
        iDynTree::MatrixDynSize m_D;                   ///< feedthrough matrix

        iDynTree::MatrixDynSize m_P;                   ///< state covariance matrix
        iDynTree::MatrixDynSize m_P0;                  ///< initial state covariance matrix
        iDynTree::MatrixDynSize m_Q;                   ///< system noise covariance matrix
        iDynTree::MatrixDynSize m_R;                   ///< measurement covariance matrix

        bool m_is_initialized{false};                      ///< flag to check if filter is initialized

        bool m_filter_constructed{false};                 ///< flag to check if the filter is constructed properly with the A, B, C, D matrices
        bool m_initial_state_set{false};                        ///< flag to check if initial state is set
        bool m_initial_state_covariance_set{false};               ///< flag to check if initial state covariance matrix is set
        bool m_measurement_noise_covariance_matrix_set{false};  ///< flag to check if measurement noise covariance matrix is set
        bool m_system_noise_covariance_matrix_set{false};       ///< flag to check if system noise covariance matrix is set

        bool m_measurement_updated{false};                 ///< flag to check if measurement is updated
        bool m_input_updated{false};                       ///< flag to check if input is updated

        bool m_use_feed_through{false};                    ///< toggle to use feed through matrix D
        bool m_use_control_input{false};                    ///< toggle to use control input matrix B
    };
}

#endif
