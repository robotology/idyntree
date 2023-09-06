// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/KalmanFilter.h>

iDynTree::DiscreteKalmanFilterHelper::DiscreteKalmanFilterHelper()
{

}

bool iDynTree::DiscreteKalmanFilterHelper::constructKalmanFilter(const iDynTree::MatrixDynSize& A,
                                                         const iDynTree::MatrixDynSize& B,
                                                         const iDynTree::MatrixDynSize& C)
{
    m_use_feed_through = false;
    iDynTree::MatrixDynSize D;
    return constructKalmanFilter(A, B, C, D);
}

bool iDynTree::DiscreteKalmanFilterHelper::constructKalmanFilter(const iDynTree::MatrixDynSize& A, const iDynTree::MatrixDynSize& C)
{
    m_use_control_input = false;
    m_use_feed_through = false;
    iDynTree::MatrixDynSize B, D;
    return constructKalmanFilter(A, B, C, D);
}

bool iDynTree::DiscreteKalmanFilterHelper::constructKalmanFilter(const iDynTree::MatrixDynSize& A, const iDynTree::MatrixDynSize& B, const iDynTree::MatrixDynSize& C, const iDynTree::MatrixDynSize& D)
{
    if (A.rows() != A.cols())
    {
        iDynTree::reportError("DiscreteKalmanFilterHelper", "constructKalmanFilter", "Could not construct KF - ill-formed state transition matrix A, exiting.");
        return false;
    }
    m_dim_X = A.rows();
    m_A.resize(m_dim_X, m_dim_X);
    m_A = A;
    m_P.resize(m_dim_X, m_dim_X);
    m_P.zero();
    m_Q.resize(m_dim_X, m_dim_X);
    m_Q.zero();

    if (B.capacity() ==  0)
    {
        m_use_control_input = false;
    }
    else if (B.rows() != A.rows())
    {
        iDynTree::reportError("DiscreteKalmanFilterHelper", "constructKalmanFilter", "Could not construct KF -  control input matrix B dimension mismatch, exiting.");
        return false;
    }
    else
    {
        m_use_control_input = true;
        m_B = B;
    }

    m_dim_U = m_B.cols();
    m_u.resize(m_dim_U);
    m_u.zero();

    if (C.capacity() == 0 || (C.cols() != A.rows()))
    {
        iDynTree::reportError("DiscreteKalmanFilterHelper", "constructKalmanFilter", "Could not construct KF -  output matrix C dimension mismatch, exiting.");
        return false;
    }
    m_dim_Y = C.rows();
    m_C = C;
    m_y.resize(m_dim_Y);
    m_y.zero();
    m_R.resize(m_dim_Y, m_dim_Y);
    m_R.zero();

    if (D.capacity() ==  0)
    {
        m_use_feed_through = false;
    }
    else if ((D.rows() != C.rows()) || (D.cols() != B.rows()))
    {
        iDynTree::reportError("DiscreteKalmanFilterHelper", "constructKalmanFilter", "Could not construct KF -  control feedthrough matrix D dimension mismatch, exiting.");
        return false;
    }
    else
    {
        m_use_feed_through = true;
        m_D = D;
    }

    m_filter_constructed = true;
    return true;
}

bool iDynTree::DiscreteKalmanFilterHelper::kfSetInitialState(const iDynTree::VectorDynSize& x0)
{
    if (x0.size() != m_dim_X)
    {
        iDynTree::reportError("DiscreteKalmanFilterHelper", "kfSetInitialState", "Could not set KF initial state - dimension mismatch");
        return false;
    }

    m_x = m_x0 = x0;
    m_initial_state_set = true;
    return true;
}

bool iDynTree::DiscreteKalmanFilterHelper::kfSetStateCovariance(const iDynTree::MatrixDynSize& P)
{
    if ( (P.rows() != m_dim_X) || (P.cols() != m_dim_X))
    {
        iDynTree::reportError("DiscreteKalmanFilterHelper", "kfSetStateCovariance", "Could not set KF initial state covariance - dimension mismatch");
        return false;
    }

    m_P = m_P0 = P;
    m_initial_state_covariance_set = true;
    return true;
}

bool iDynTree::DiscreteKalmanFilterHelper::kfSetSystemNoiseCovariance(const iDynTree::MatrixDynSize& Q)
{
    if ( (Q.rows() != m_dim_X) || (Q.cols() != m_dim_X))
    {
        iDynTree::reportError("DiscreteKalmanFilterHelper", "kfSetSystemNoiseCovariance", "Could not set KF system noise covariance - dimension mismatch");
        return false;
    }

    m_Q = Q;
    m_system_noise_covariance_matrix_set = true;
    return true;
}

bool iDynTree::DiscreteKalmanFilterHelper::kfSetMeasurementNoiseCovariance(const iDynTree::MatrixDynSize& R)
{
    if ( (R.rows() != m_dim_Y) || (R.cols() != m_dim_Y))
    {
        iDynTree::reportError("DiscreteKalmanFilterHelper", "kfSetMeasurementNoiseCovariance", "Could not set KF measurement noise covariance - dimension mismatch");
        return false;
    }

    m_R = R;
    m_measurement_noise_covariance_matrix_set = true;
    return true;
}

bool iDynTree::DiscreteKalmanFilterHelper::kfInit()
{
    if (!m_filter_constructed)
    {
        iDynTree::reportError("DiscreteKalmanFilterHelper", "kfInit", "Please construct the filter first.");
        return false;
    }

    if (!m_measurement_noise_covariance_matrix_set || !m_system_noise_covariance_matrix_set)
    {
        iDynTree::reportError("DiscreteKalmanFilterHelper", "kfInit", "Please set the noise covariance matrices Q and R.");
        return false;
    }

    if (!m_initial_state_set || !m_initial_state_covariance_set)
    {
        iDynTree::reportError("DiscreteKalmanFilterHelper", "kfInit", "Please set the initial state and the state covariance matrix.");
        return false;
    }

    m_is_initialized = true;
    return true;
}

bool iDynTree::DiscreteKalmanFilterHelper::kfSetInputVector(const iDynTree::VectorDynSize& u)
{
    if (!m_use_control_input)
    {
        iDynTree::reportError("DiscreteKalmanFilterHelper", "kfSetInput", "B matrix was not constructed properly");
        return false;
    }

    if (u.size() != m_dim_U)
    {
        iDynTree::reportError("DiscreteKalmanFilterHelper", "kfSetInput", "Could not set inputs - dimensions mismatch");
        return false;
    }

    m_u = u;
    m_input_updated = true;
    return true;
}

bool iDynTree::DiscreteKalmanFilterHelper::kfPredict()
{
    if (!m_is_initialized)
    {
        iDynTree::reportError("DiscreteKalmanFilterHelper", "kfPredict", "filter not initialized.");
        return false;
    }

    if (m_use_control_input)
    {
        if (!m_input_updated)
        {
            iDynTree::reportWarning("DiscreteKalmanFilterHelper", "kfPredict", "input not updated. using old input");
        }
    }

    using iDynTree::toEigen;
    auto x(toEigen(m_x));
    auto P(toEigen(m_P));
    auto A(toEigen(m_A));
    auto Q(toEigen(m_Q));

    if (m_use_control_input)
    {
        auto B(toEigen(m_B));
        auto u(toEigen(m_u));
        x = (A*x) + (B*u);
    }
    else
    {
        x = A*x;
    }

    // propagate covariance
    P = A*P*(A.transpose()) + Q;
    m_input_updated = false;
    return true;
}

bool iDynTree::DiscreteKalmanFilterHelper::kfSetMeasurementVector(const iDynTree::VectorDynSize& y)
{
    if (y.size() != m_dim_Y)
    {
        iDynTree::reportError("DiscreteKalmanFilterHelper", "kfSetInitialState", "Could not set measurement - dimension mismatch");
        return false;
    }

    m_y = y;
    m_measurement_updated = true;
    return true;
}

bool iDynTree::DiscreteKalmanFilterHelper::kfUpdate()
{
    if (!m_is_initialized)
    {
        iDynTree::reportError("DiscreteKalmanFilterHelper", "kfUpdate", "filter not initialized.");
        return false;
    }

    if (!m_measurement_updated)
    {
        iDynTree::reportError("DiscreteKalmanFilterHelper", "kfUpdate", "measurements not updated.");
        return false;
    }

    using iDynTree::toEigen;
    iDynTree::VectorDynSize z_;
    z_.resize(m_dim_Y);
    auto z(toEigen(z_));
    auto C(toEigen(m_C));
    auto x(toEigen(m_x));
    auto y(toEigen(m_y));
    auto P(toEigen(m_P));
    auto R(toEigen(m_R));

    if (m_use_feed_through)
    {
        auto D(toEigen(m_D));
        auto u(toEigen(m_u));
        z = (C*x) + (D*u);
    }
    else
    {
        z = C*x;
    }

    auto innovation = y - z;
    auto I =Eigen::MatrixXd::Identity(m_dim_X, m_dim_X);
    auto S = C*P*(C.transpose()) + R;
    auto K = P*(C.transpose())*(S.inverse());
    x = x + (K*innovation);
    P = (I - (K*C))*P;
    m_measurement_updated = false;
    return true;
}

bool iDynTree::DiscreteKalmanFilterHelper::kfReset()
{
    return kfReset(m_x0, m_P0, m_Q, m_R);
}

bool iDynTree::DiscreteKalmanFilterHelper::kfReset(const iDynTree::VectorDynSize& x0,
                                                   const iDynTree::MatrixDynSize& P0,
                                                   const iDynTree::MatrixDynSize& Q,
                                                   const iDynTree::MatrixDynSize& R)
{
    m_initial_state_set = m_initial_state_covariance_set = false;
    m_measurement_noise_covariance_matrix_set = m_system_noise_covariance_matrix_set = false;
    kfSetInitialState(x0);
    kfSetStateCovariance(P0);
    kfSetSystemNoiseCovariance(Q);
    kfSetMeasurementNoiseCovariance(R);

    return kfInit();
}

bool iDynTree::DiscreteKalmanFilterHelper::kfGetStateCovariance(iDynTree::MatrixDynSize& P)
{
    P.resize(m_P.rows(), m_P.cols());
    P = m_P;
    return true;
}

bool iDynTree::DiscreteKalmanFilterHelper::kfGetStates(iDynTree::VectorDynSize& x)
{
    x.resize(m_x.size());
    x = m_x;
    return true;
}
