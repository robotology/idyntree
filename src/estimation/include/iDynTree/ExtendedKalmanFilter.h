// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

#include <iDynTree/VectorDynSize.h>
#include <iDynTree/MatrixDynSize.h>
#include <iDynTree/Utils.h>
#include <vector>

namespace iDynTree
{

    /**
     * @class DiscreteExtendedKalmanFilterHelper naive base class implementation of discrete EKF with additive Gaussian noise
     *
     * @warning This class is not stand-alone but can be used meaningfully only by classes deriving from it.
     * The system propagation function f() and measurement model function h() are virtual functions to be defined by
     * the derived class implementing the EKF.
     * Similarly, the Jacobians for the linearized system propagation and measurement model are also virtual functions
     * to be implemented by the derived class.
     *
     * The derived class must must set the size for states, inputs and outputs using ekfSetStateSize(), ekfSetInputSize() and ekfSetOutputSize() methods.
     * Then the derived class must call ekfInit() to resize the vectors and matrices for the EKF.
     *
     * Similarly before running the estimator through a loop, it is recommended to set the initial states and variances for measurements, system dynamics and intial states.
     * This is necessary to avoid any NaN values to be propagated.
     *
     * Once, initialized properly, the filter can be run by
     *  - calling ekfSetInputVector() to set the control inputs and then calling ekfPredict() at each prediction step, and
     *  - calling ekfSetMeasurementVector() to set the measurements and then calling ekfUpdate() at each update step
     *
     * The internal state of the estimator can be obtained by calling ekfGetStates().
     *
     * The Discrete Extended Kalman Filter equations implemented in this class are coherent with the ones described in
     * <a href="https://en.wikipedia.org/wiki/Extended_Kalman_filter">Discrete-time predict and update equations section of this article.</a>
     *
     * The general workflow implementing/inheriting this class would be in the order,
     * - call ekfSetInputSize(), ekfSetOutputSize(), ekfSetStateSize() methods
     * - call ekfInit()
     * - call ekfSetInitialState(), ekfSetStateCovariance() (either done externally later or internally. usually done externally later, however, filter runs properly only if this step is done)
     * - call ekfSetSystemNoiseCovariance(), ekfSetMeasurementNoiseCovariance()
     *
     *   once this is setup,
     * - in a loop
     *     - call ekfSetInputVector() then ekfPredict(). Calling ekfGetStates() and ekfGetStateCovariance() at this point will give us the predicted states and its covariance
     *     - call ekfSetMeasurementVector() then ekfUpdate(). Calling ekfGetStates() and ekfGetStateCovariance() at this point will give us the updated states and its covariance
     * @note if we intend to change the state size, input size or output size on the fly, it is crucial to call the ekfInit() method again, since this resizes the buffers accordingly
     *       failing to do so will result in memory leaks and will cause the program to crash.
     *
     */
    class DiscreteExtendedKalmanFilterHelper
    {
    public:
        DiscreteExtendedKalmanFilterHelper();

        /**
         * @brief Describes the state propagation for a given dynamical system
         *        If state of the system is denoted by \f$ x \f$ and the control input by \f$ u \f$, then
         *        the system dynamics is given as \f$ x_{k+1} = f(x_k, u_k) \f$.
         * @note the detail of this function needs to be implemented by the child class
         * @param[in] x_k state at current time instant
         * @param[in] u_k control input at current time instant
         * @param[out] xhat_k_plus_one predicted state without any correction from measurements
         * @return bool true/false if successful or not
         */
        virtual bool ekf_f(const iDynTree::VectorDynSize& x_k,
                       const iDynTree::VectorDynSize& u_k,
                             iDynTree::VectorDynSize& xhat_k_plus_one) = 0;

        /**
         * @brief Describes the measurement model of the system,
         *        i.e., how the measurements can be described as a function of states,
         *        Given a state of the system described by \f$ x \f$,
         *        what would be the measurement \f$ z \f$ observed from this state \f$ z_{k+1} = h(\hat{x}_{k+1}) \f$.
         * @note the detail of this function needs to be implemented by the child class
         * @param[in] xhat_k_plus_one predicted state of next time instant
         * @param[out] zhat_k_plus_one predicted measurement of next time instant
         * @return bool true/false if successful or not
         */
        virtual bool ekf_h(const iDynTree::VectorDynSize& xhat_k_plus_one,
                             iDynTree::VectorDynSize& zhat_k_plus_one) = 0;

        /**
         * @overload
         */
        virtual bool ekfComputeJacobianF(iDynTree::VectorDynSize& x, iDynTree::MatrixDynSize& F) = 0;

        /**
         * @brief Describes the system Jacobian necessary for the propagation of predicted state covariance
         *        The analytical Jacobian describing the partial derivative of the system propagation with respect to the state
         *        and the system propagation with respect to the input
         * @note the detail of this function needs to be implemented by the child class
         * @param[in] x system state
         * @param[in] u system input
         * @param[out] F system Jacobian
         * @return bool true/false if successful or not
         */
        virtual bool ekfComputeJacobianF(iDynTree::VectorDynSize& x, iDynTree::VectorDynSize& u,  iDynTree::MatrixDynSize& F) = 0;

        /**
         * @brief Describes the measurement Jacobian necessary for computing Kalman gain and updating the predicted state and its covariance
         *        The analytical Jacobian describing the partial derivative of the measurement model with respect to the state
         * @note the detail of this function needs to be implemented by the child class
         * @param[in] x system state
         * @param[out] H measurement Jacobian
         * @return bool true/false if successful or not
         */
        virtual bool ekfComputeJacobianH(iDynTree::VectorDynSize& x, iDynTree::MatrixDynSize& H) = 0;

        /**
         * @brief Implements the Discrete EKF prediction equation
         *        described by
         *        \f$ \hat{x}_{k+1} = f(x_k, u_k) \f$ is given by the ekf_f() method
         *        \f$ \hat{P}_{k+1} = F_k P_k F_k^T + Q \f$
         *        where, \f$ F \mid_{x = x_k} \f$ is given by the ekfComputejacobianF() method
         *
         * @warning this function can be called only after setting up the filter properly through ekfInit() step
         * @note this function should be once called every step, after setting up the input vector using ekfSetInputVector() method
         * @warning setting up the input vector everytime before calling the ekfPredict() method is crucial, the prediction step is not performed if this step is skipped
         * this is because internally a flag associated to the setting up of input vector is set true by ekfSetInputVector() method which in turn is set false by ekfPredict() method
         * @return bool true/false if successful or not
         */
        bool ekfPredict();

        /**
         * @brief Implements the Discrete EKF update equation
         *        described by
         *        \f$ z_{k+1} = h(\hat{x}_{k+1}) \f$ is given by ekf_h() method
         *        innovation \f$ \tilde{y}_{k+1} = y_{k+1} - z_{k+1} \f$
         *        innovation covariance \f$ S_{k+1} = H_{k+1} \hat{P}_{k+1} H_{k+1}^T + R \f$, where \f$ H \mid_{x = \hat{x}_{k+1}} \f$ is given by ekfComputejacobianH() method
         *        Kalman gain \f$ K_{k+1} = \hat{P}_{k+1} H_{k+1}^T S_{k+1}^{-1} \f$
         *        Updated covariance \f$ P_{k+1} = \hat{P}_{k+1} - (K_{k+1} H \hat{P}_{k+1}) \f$
         *        Updated state estimate \f$ x_{k+1} = \hat{x}_{k+1} + K_{k+1} \tilde{y}_{k+1} \f$
         *
         * @warning this function can be called only after setting up the filter properly through ekfInit() step
         * @note this function should be once called every step, after setting up the measurement vector using ekfSetMeasurementVector() method
         * @warning setting up the measurement vector everytime before calling the ekfUpdate() method is crucial, the update step is not performed if this step is skipped
         * this is because internally a flag associated to the setting up of measurement vector is set true by ekfSetMeasurementVector() method which in turn is set false by ekfUpdate() method
         * @return bool true/false if successful or not
         */
        bool ekfUpdate();

        /**
         * @brief Initializes and resizes the internal buffers of this filter
         * @warning this is a very crucial method of this class. This needs to be called after setting the input size through ekfSetInputSize(),
         *          output size through ekfSetOutputSize() and state dimension through ekfSetStateSize(),
         *          such that the corresponding matrices and vectors will resize themselves to their corresponding dimensions.
         *          Failing to do so might result in memory leaks and may cause the program to crash
         * @return bool true/false if successful or not
         */
        bool ekfInit();

        /**
         * @brief Initializes and resizes the internal buffers of this filter
         * @warning this is a very crucial method of this class. This method sets the input size through ekfSetInputSize(),
         *          output size through ekfSetOutputSize() and state dimension through ekfSetStateSize() with the specified parameters,
         *          such that the corresponding matrices and vectors will resize themselves to their corresponding dimensions.
         *          Failing to do so might result in memory leaks and may cause the program to crash
         * @param[in] state_size state size
         * @param[in] input_size input size
         * @param[in] output_size output size
         * @return bool true/false if successful or not
         */
        bool ekfInit(const size_t& state_size, const size_t& input_size, const size_t& output_size);

        /**
         * @brief Resets the filter flags
         * The filter flags check if the filter was properly initialized, if the initial state was set,
         * if the initial state covariance was set. These three flags are crucial for proper setting up of the filter.
         * The other flags include the checks on whether the input and measurement vectors were updated at every prediction/update step
         */
        void ekfReset();

        /**
         * @brief Resets the filter flags, initializes and resizes internal buffers of the filter, and
         *             sets initial state, initial state covariance, and system noise and measurement noise covariance matrices
         * @warning size of the span for P0 and Q must be of the size (state size*state size), where * is the regular multiplication operator
         * @warning size of the span for R must be of the size (ouput size*output size),  where * is the regular multiplication operator
         * @warning the matrices from the span are built in row-major ordering.
         *
         * @note this method is particularly useful while working with hybrid systems,
         *             where the size of the system state or the measurements keep evolving with time
         *
         */
        bool ekfReset(const size_t& state_size,
                                 const size_t& input_size,
                                 const size_t& output_size,
                                 const iDynTree::Span<double>& x0,
                                 const iDynTree::Span<double>& P0,
                                 const iDynTree::Span<double>& Q,
                                 const iDynTree::Span<double>& R);

        /**
         * @brief Set measurement vector at every time step
         * the measurement vector size and output size should match
         * @param[in] y iDynTree::Span object to access the measurement vector
         * @return bool true/false if successful or not
         */
        bool ekfSetMeasurementVector(const iDynTree::Span<double>& y);

        /**
         * @brief Set input vector at every time step
         * the input vector size and input size should match
         * @param[in] u iDynTree::Span object to access the input vector
         * @return bool true/false if successful or not
         */
        bool ekfSetInputVector(const iDynTree::Span<double>& u);

        /**
         * @brief Set initial state
         * the size of x0 and state size should match
         * @param[in] x0 iDynTree::Span object to access the state vector
         * @note this method should be called before running the filter
         * @return bool true/false if successful or not
         */
        bool ekfSetInitialState(const iDynTree::Span<double>& x0);

        /**
         * @brief Set initial state covariance matrix
         * the size of P and (state size*state size) should match
         * @param[in] P iDynTree::Span object to access the state covariance matrix
         * @note this method should be called before running the filter
         * @warning if this matrix is not initialized properly, then the resulting output will only have NaNs in it
         * @return bool true/false if successful or not
         */
        bool ekfSetStateCovariance(const iDynTree::Span<double>& P);

        /**
         * @brief Set system noise covariance matrix
         * the size of Q and (state size*state size) should match
         * @param[in] Q iDynTree::Span object to access the system noise covariance matrix
         * @note default value is a zero matrix
         * @return bool true/false if successful or not
         */
        bool ekfSetSystemNoiseCovariance(const iDynTree::Span<double>& Q);

        /**
         * @brief Set measurement noise covariance matrix
         * the size of R and (output size*output size) should match
         * @param[in] R iDynTree::Span object to access the measurement noise covariance matrix
         * @note default value is a zero matrix
         * @return bool true/false if successful or not
         */
        bool ekfSetMeasurementNoiseCovariance(const iDynTree::Span<double>& R);

        /**
         * @brief Set the state dimensions
         * @param[in] dim_X state size
         * @warning this method should be called before calling ekfInit()
         * @return bool true/false if successful or not
         */
        void ekfSetStateSize(size_t dim_X) { m_dim_X = dim_X; }

        /**
         * @brief Set the input dimensions
         * @param[in] dim_U input size
         * @warning this method should be called before calling ekfInit()
         * @return bool true/false if successful or not
         */
        void ekfSetInputSize(size_t dim_U) { m_dim_U = dim_U; }

        /**
         * @brief Set the ouptut dimensions
         * @param[in] dim_Y output size
         * @warning this method should be called before calling ekfInit()
         * @return bool true/false if successful or not
         */
        void ekfSetOutputSize(size_t dim_Y) { m_dim_Y = dim_Y; }

        /**
         * @brief Get current internal state of the filter
         * the size of x and state size should match
         * @param[out] x iDynTree::Span object to copy the internal state vector into
         * @return bool true/false if successful or not
         */
        bool ekfGetStates(const iDynTree::Span<double> &x) const;

        /**
         * @brief Get state covariance matrix
         * the size of P and (state size*state size) should match
         * @param[out] P iDynTree::Span object to copy the internal state covariance matrix onto
         * @return bool true/false if successful or not
         */
        bool ekfGetStateCovariance(const iDynTree::Span<double> &P) const;

   protected:
        /**
        * function template to ignore unused parameters
        */
        template <typename T>
        void ignore(T &&) { }

    private:
        size_t m_dim_X;                                ///< state dimension
        size_t m_dim_Y;                                ///< output dimenstion
        size_t m_dim_U;                                ///< input dimension
        iDynTree::VectorDynSize m_x;                   ///< state at time instant k
        iDynTree::VectorDynSize m_u;                   ///< input at time instant k
        iDynTree::VectorDynSize m_y;                   ///< measurements at time instant k
        iDynTree::VectorDynSize m_xhat;                ///< predicted state at time instant k before updating measurements

        iDynTree::MatrixDynSize m_F;                   ///< System jacobian
        iDynTree::MatrixDynSize m_P;                   ///< State covariance
        iDynTree::MatrixDynSize m_Phat;                ///< State covariance estimate before updating measurements
        iDynTree::MatrixDynSize m_Q;                   ///< system noise covariance
        iDynTree::MatrixDynSize m_H;                   ///< measurement jacobian
        iDynTree::MatrixDynSize m_S;                   ///< innovation covariance
        iDynTree::MatrixDynSize m_K;                   ///< Kalman gain
        iDynTree::MatrixDynSize m_R;                   ///< measurement noise covariance
        bool m_is_initialized{false};                  ///< flag to check if filter is properly initialized
        bool m_measurement_updated{false};             ///< flag to check if measurement is updated at each update step
        bool m_input_updated{false};                   ///< flag to check if control input is updated at each prediction step
        bool m_initial_state_set{false};               ///< flag to check if the initial state of the filter is set
        bool m_initial_state_covariance_set{false};    ///< flag to check if the initial covariance is set properly
    };
}

#endif
