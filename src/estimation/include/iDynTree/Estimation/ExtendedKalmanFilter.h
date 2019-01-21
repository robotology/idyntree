/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/EigenHelpers.h>

namespace iDynTree
{

    /**
     * @class DiscreteExtendedKalmanFilter naive base class implementation of discrete EKF with additive Gaussian noise
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
     * The internal state of the estimator can be obtained by calling ekfGetStates()
     */
    class DiscreteExtendedKalmanFilter
    {
    protected:
        DiscreteExtendedKalmanFilter();
        virtual bool f(const iDynTree::VectorDynSize& x_k,
                       const iDynTree::VectorDynSize& u_k,
                       const iDynTree::VectorDynSize& w_k,
                       iDynTree::VectorDynSize& xhat_k_plus_one) = 0;

        virtual bool h(const iDynTree::VectorDynSize& xhat_k_plus_one,
                       const iDynTree::VectorDynSize& v_k_plus_one,
                       iDynTree::VectorDynSize& zhat_k_plus_one) = 0;

        virtual bool computejacobianF(iDynTree::VectorDynSize& x, iDynTree::MatrixDynSize& F) = 0;
        virtual bool computejacobianH(iDynTree::VectorDynSize& x, iDynTree::MatrixDynSize& H) = 0;

        bool ekfPredict();
        bool ekfUpdate();
        bool ekfInit();
        void ekfReset();

        bool ekfSetMeasurementVector(const iDynTree::Span<double>& y);
        bool ekfSetInputVector(const iDynTree::Span<double>& u);
        bool ekfSetInitialState(const iDynTree::Span<double>& x0);
        bool ekfSetStateCovariance(const iDynTree::Span<double>& P);
        bool ekfSetSystemNoiseMeanAndCovariance(const iDynTree::Span<double>& w_mean, const iDynTree::Span<double>& Q);
        bool ekfSetMeasurementNoiseMeanAndCovariance(const iDynTree::Span<double>& v_mean, const iDynTree::Span<double>& R);
        void ekfSetStateSize(size_t dim_X) { m_dim_X = dim_X; }
        void ekfSetInputSize(size_t dim_U) { m_dim_U = dim_U; }
        void ekfSetOutputSize(size_t dim_Y) { m_dim_Y = dim_Y; }

        bool ekfGetStates(iDynTree::Span<double> &x) const;
//         bool ekfGetStateCovariance(iDynTree::Span<double> &P) const;

    private:
        size_t m_dim_X; ///< state dimension
        size_t m_dim_Y; ///< output dimenstion
        size_t m_dim_U; ///< input dimension
        iDynTree::VectorDynSize m_x;  ///< state at time instant k
        iDynTree::VectorDynSize m_u;  ///< input at time instant k
        iDynTree::VectorDynSize m_w;  ///< system noise at time instant k
        iDynTree::VectorDynSize m_y;  ///< measurements at time instant k
        iDynTree::VectorDynSize m_xhat; ///< predicted state at time instant k before updating measurements
        iDynTree::VectorDynSize m_v;  ///< measurement noise at time instant k+1

        iDynTree::MatrixDynSize m_F; ///< System jacobian
        iDynTree::MatrixDynSize m_P; ///< State covariance
        iDynTree::MatrixDynSize m_Phat; ///< State covariance estimate before updating measurements
        iDynTree::MatrixDynSize m_Q; ///< system noise covariance
        iDynTree::MatrixDynSize m_H; ///< measurement jacobian
        iDynTree::MatrixDynSize m_S; ///< innovation covariance
        iDynTree::MatrixDynSize m_K; ///< Kalman gain
        iDynTree::MatrixDynSize m_R; ///< measurement noise covariance
        bool m_is_initialized{false}; ///< flag to check if filter is properly initialized
        bool m_measurement_updated{false}; ///< flag to check if measurement is updated at each update step
        bool m_input_updated{false}; ///< flag to check if control input is updated at each prediction step
        bool m_initial_state_set{false};  ///< flag to check if the initial state of the filter is set
        bool m_initial_state_covariance_set{false}; ///< flag to check if the initial covariance is set properly
    };
}

#endif
