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
    // naive implementation of discrete EKF with additive Gaussian noise
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
        //iDynTree::VectorDynSize m_y_k_plus_one;  ///< measurements at time instant k+1
        iDynTree::VectorDynSize m_v;  ///< measurement noise at time instant k+1

        iDynTree::MatrixDynSize m_F; ///< System jacobian
        iDynTree::MatrixDynSize m_P; ///< System covariance
        iDynTree::MatrixDynSize m_Phat; ///< System covariance estimate
        iDynTree::MatrixDynSize m_Q; ///< system noise covariance
        iDynTree::MatrixDynSize m_H; ///< measurement jacobian
        iDynTree::MatrixDynSize m_S; ///< used to compute gain
        iDynTree::MatrixDynSize m_K; ///< Kalman gain
        iDynTree::MatrixDynSize m_R; ///< measurement noise covariance
        bool m_is_initialized{false};
        bool m_measurement_updated{false};
        bool m_input_updated{false};
        bool m_initial_state_set{false};
        bool m_initial_state_covariance_set{false};
    };
}

#endif
