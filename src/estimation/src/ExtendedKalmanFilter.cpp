/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Estimation/ExtendedKalmanFilter.h>

iDynTree::DiscreteExtendedKalmanFilter::DiscreteExtendedKalmanFilter()
{

}

void iDynTree::DiscreteExtendedKalmanFilter::ekfReset()
{
    m_is_initialized = false;
    m_measurement_updated = false;
    m_input_updated = false;
    m_initial_state_set = false;
    m_initial_state_covariance_set = false;
}


bool iDynTree::DiscreteExtendedKalmanFilter::ekfInit()
{
    if (m_dim_X == 0 || m_dim_Y == 0 || m_dim_U == 0)
    {
        iDynTree::reportError("DiscreteExtendedKalmanFilter", "init", "state or input or output size not set, exiting.");
        return false;
    }

    // resize state related quantities
    m_x.resize(m_dim_X); m_x.zero();
    m_xhat.resize(m_dim_X); m_xhat.zero();
    m_u.resize(m_dim_U); m_u.zero();
    m_y.resize(m_dim_Y); m_y.zero();

    m_w.resize(m_dim_X); m_w.zero();
    m_v.resize(m_dim_Y); m_v.zero();

    m_F.resize(m_dim_X, m_dim_X);
    m_F.zero();
    m_P.resize(m_dim_X, m_dim_X);
    m_P.zero();
    m_Phat.resize(m_dim_X, m_dim_X);
    m_Phat.zero();
    m_Q.resize(m_dim_X, m_dim_X);
    m_Q.zero();

    m_H.resize(m_dim_Y, m_dim_X);
    m_H.zero();
    m_S.resize(m_dim_Y, m_dim_Y);
    m_S.zero();
    m_K.resize(m_dim_X, m_dim_Y);
    m_K.zero();
    m_R.resize(m_dim_Y, m_dim_Y);
    m_R.zero();

    m_is_initialized = true;
    return m_is_initialized;

}

bool iDynTree::DiscreteExtendedKalmanFilter::ekfPredict()
{
    if (!m_is_initialized)
    {
        iDynTree::reportError("DiscreteExtendedKalmanFilter", "ekfPredict", "filter not initialized.");
        return false;
    }

    if (!m_initial_state_set)
    {
        iDynTree::reportError("DiscreteExtendedKalmanFilter", "ekfPredict", "initial state not set.");
        return false;
    }

    if (!m_initial_state_covariance_set)
    {
        iDynTree::reportError("DiscreteExtendedKalmanFilter", "ekfPredict", "initial state covariance not set.");
        return false;
    }

    if (!m_input_updated)
    {
        iDynTree::reportWarning("DiscreteExtendedKalmanFilter", "ekfPredict", "input not updated. using old input");
    }

    f(m_x, m_u, m_w, m_xhat); // xhat_k+1 = f(x_k, u_k) + w_k
    computejacobianF(m_x, m_F); // F at x = x_k
    iDynTree::toEigen(m_Phat) = iDynTree::toEigen(m_F)*iDynTree::toEigen(m_P)*(iDynTree::toEigen(m_F).transpose()) + iDynTree::toEigen(m_Q); // Phat_k+1 = F_k P_k (F_k)^T + Q
    m_input_updated = false;
    return true;
}

bool iDynTree::DiscreteExtendedKalmanFilter::ekfUpdate()
{
    if (!m_is_initialized)
    {
        iDynTree::reportError("DiscreteExtendedKalmanFilter", "ekfUpdate", "filter not initialized.");
        return false;
    }

    if (!m_initial_state_set)
    {
        iDynTree::reportError("DiscreteExtendedKalmanFilter", "ekfUpdate", "initial state not set.");
        return false;
    }

    if (!m_initial_state_covariance_set)
    {
        iDynTree::reportError("DiscreteExtendedKalmanFilter", "ekfUpdate", "initial state covariance not set.");
        return false;
    }

    if (!m_measurement_updated)
    {
        iDynTree::reportError("DiscreteExtendedKalmanFilter", "ekfUpdate", "measurements not updated.");
        return false;
    }

    iDynTree::VectorDynSize z;
    z.resize(m_dim_Y);
    h(m_xhat, m_v, z); // z_k+1 = h(xhat_k+1, v_k+1)
    computejacobianH(m_xhat, m_H); // H at x = xhat_k+1
    iDynTree::toEigen(m_S) = iDynTree::toEigen(m_H)*iDynTree::toEigen(m_Phat)*iDynTree::toEigen(m_H).transpose() + iDynTree::toEigen(m_R); // S = H_k+1 Phat_k+1 (H_k+1)^T + R
    iDynTree::toEigen(m_K) = iDynTree::toEigen(m_Phat)*iDynTree::toEigen(m_H).transpose()*iDynTree::toEigen(m_S).inverse(); // K_k+1 = Phat_k+1 (H_k+1)^T (S^{-1})
    iDynTree::toEigen(m_P) = iDynTree::toEigen(m_Phat) - (iDynTree::toEigen(m_K)*iDynTree::toEigen(m_S)*iDynTree::toEigen(m_K).transpose()); // P_k+1 = Phat_k+1 - K_k+1 S (K_k+1)^T
    iDynTree::toEigen(m_x) = iDynTree::toEigen(m_xhat) + iDynTree::toEigen(m_K)*(iDynTree::toEigen(m_y)-iDynTree::toEigen(z)); // x_k+1 = xhat_k+1 + K_k+1(y_k+1 - z_k+1)

    m_measurement_updated = false;
    return true;
}

bool iDynTree::DiscreteExtendedKalmanFilter::ekfSetInitialState(const iDynTree::Span< double >& x0)
{
    if ((size_t)x0.size() != m_dim_X)
    {
        iDynTree::reportError("DiscreteExtendedKalmanFilter", "setInitialState", "state size mismatch");
        return false;
    }

    for (int i = 0; i < x0.size(); i++)
    {
        m_x(i) = x0(i);
    }

    m_initial_state_set = true;
    return true;
}

bool iDynTree::DiscreteExtendedKalmanFilter::ekfSetInputVector(const iDynTree::Span< double >& u)
{
    if ((size_t)u.size() != m_dim_U)
    {
        iDynTree::reportError("DiscreteExtendedKalmanFilter", "ekfSetInputVector", "input size mismatch");
        return false;
    }

    for (int i = 0; i < u.size(); i++)
    {
        m_u(i) = u(i);
    }

    m_input_updated = true;
    return true;
}


bool iDynTree::DiscreteExtendedKalmanFilter::ekfSetMeasurementVector(const iDynTree::Span< double >& y)
{
    if ((size_t)y.size() != m_dim_Y)
    {
        iDynTree::reportError("DiscreteExtendedKalmanFilter", "ekfSetMeasurementVector", "measurement size mismatch");
        return false;
    }

    for (int i = 0; i < y.size(); i++)
    {
        m_y(i) = y(i);
    }

    m_measurement_updated = true;
    return true;
}

bool iDynTree::DiscreteExtendedKalmanFilter::ekfSetStateCovariance(const iDynTree::Span< double >& P)
{
    if ((size_t)P.size() != m_dim_X*m_dim_X)
    {
        iDynTree::reportError("DiscreteExtendedKalmanFilter", "setSystemCovariance", "state covariance matrix size mismatch");
        return false;
    }

    m_P = iDynTree::MatrixDynSize(P.data(), m_dim_X, m_dim_X);
    m_initial_state_covariance_set = true;
    return true;
}

bool iDynTree::DiscreteExtendedKalmanFilter::ekfSetSystemNoiseMeanAndCovariance(const iDynTree::Span< double >& w_mean, const iDynTree::Span< double >& Q)
{
    if ((size_t)w_mean.size() != m_dim_X)
    {
        iDynTree::reportError("DiscreteExtendedKalmanFilter", "setSystemNoiseMeanAndCovariance", "noise vector size mismatch");
        return false;
    }

    for (int i = 0; i < w_mean.size(); i++)
    {
        m_w = w_mean;
    }

    if ((size_t)Q.size() != m_dim_X*m_dim_X)
    {
        iDynTree::reportError("DiscreteExtendedKalmanFilter", "setSystemNoiseMeanAndCovariance", "noise covariance matrix size mismatch");
        return false;
    }

    m_Q = iDynTree::MatrixDynSize(Q.data(), m_dim_X, m_dim_X);
    return true;
}


bool iDynTree::DiscreteExtendedKalmanFilter::ekfSetMeasurementNoiseMeanAndCovariance(const iDynTree::Span< double >& v_mean, const iDynTree::Span< double >& R)
{
    if ((size_t)v_mean.size() != m_dim_Y)
    {
        iDynTree::reportError("DiscreteExtendedKalmanFilter", "setMeasurementNoiseMeanAndCovariance", "noise vector size mismatch");
        return false;
    }

    for (int i = 0; i < v_mean.size(); i++)
    {
        m_v = v_mean;
    }

    if ((size_t)R.size() != m_dim_Y*m_dim_Y)
    {
        iDynTree::reportError("DiscreteExtendedKalmanFilter", "setMeasurementNoiseMeanAndCovariance", "noise covariance matrix size mismatch");
        return false;
    }

    m_R = iDynTree::MatrixDynSize(R.data(), m_dim_Y, m_dim_Y);
    return true;
}

bool iDynTree::DiscreteExtendedKalmanFilter::ekfGetStates(iDynTree::Span< double >& x) const
{
    if ((size_t)x.size() != m_dim_X)
    {
        iDynTree::reportError("DiscreteExtendedKalmanFilter", "ekfGetStates", "state size mismatch");
        return false;
    }

    for (size_t i = 0; i < m_dim_X; i++)
    {
        x(i) = m_x(i);
    }
    return true;
}
