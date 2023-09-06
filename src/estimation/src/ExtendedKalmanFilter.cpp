// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/ExtendedKalmanFilter.h>
#include <iDynTree/EigenHelpers.h>

iDynTree::DiscreteExtendedKalmanFilterHelper::DiscreteExtendedKalmanFilterHelper()
{

}

void iDynTree::DiscreteExtendedKalmanFilterHelper::ekfReset()
{
    m_is_initialized = false;
    m_measurement_updated = false;
    m_input_updated = false;
    m_initial_state_set = false;
    m_initial_state_covariance_set = false;
}

bool iDynTree::DiscreteExtendedKalmanFilterHelper::ekfInit(const size_t& state_size, const size_t& input_size, const size_t& output_size)
{
    m_dim_X = state_size;
    m_dim_U = input_size;
    m_dim_Y = output_size;

    return ekfInit();
}

bool iDynTree::DiscreteExtendedKalmanFilterHelper::ekfReset(const size_t& state_size, const size_t& input_size, const size_t& output_size,
                                                                                                        const iDynTree::Span<double>& x0, const iDynTree::Span<double>& P0,
                                                                                                        const iDynTree::Span<double>& Q, const iDynTree::Span<double>& R)
{
    m_is_initialized = false;
    m_measurement_updated = false;
    m_input_updated = false;
    m_initial_state_set = false;
    m_initial_state_covariance_set = false;

    if (!ekfInit(state_size, input_size, output_size))
    {
        return false;
    }

    if (!ekfSetInitialState(x0))
    {
        return false;
    }

    if (!ekfSetStateCovariance(P0))
    {
        return false;
    }

    if (!ekfSetSystemNoiseCovariance(Q))
    {
        return false;
    }

    if (!ekfSetMeasurementNoiseCovariance(R))
    {
        return false;
    }

    return true;
}


bool iDynTree::DiscreteExtendedKalmanFilterHelper::ekfInit()
{
    if (m_dim_X == 0 || m_dim_Y == 0 || m_dim_U == 0)
    {
        iDynTree::reportError("DiscreteExtendedKalmanFilterHelper", "init", "state or input or output size not set, exiting.");
        return false;
    }

    // resize state related quantities
    m_x.resize(m_dim_X); m_x.zero();
    m_xhat.resize(m_dim_X); m_xhat.zero();
    m_u.resize(m_dim_U); m_u.zero();
    m_y.resize(m_dim_Y); m_y.zero();

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

bool iDynTree::DiscreteExtendedKalmanFilterHelper::ekfPredict()
{
    if (!m_is_initialized)
    {
        iDynTree::reportError("DiscreteExtendedKalmanFilterHelper", "ekfPredict", "filter not initialized.");
        return false;
    }

    if (!m_initial_state_set)
    {
        iDynTree::reportError("DiscreteExtendedKalmanFilterHelper", "ekfPredict", "initial state not set.");
        return false;
    }

    if (!m_initial_state_covariance_set)
    {
        iDynTree::reportError("DiscreteExtendedKalmanFilterHelper", "ekfPredict", "initial state covariance not set.");
        return false;
    }

    if (!m_input_updated)
    {
        iDynTree::reportWarning("DiscreteExtendedKalmanFilterHelper", "ekfPredict", "input not updated. using old input");
    }

    // propagate states and compute jacobian
    ekf_f(m_x, m_u, m_xhat);                      ///< \f$ \hat{x}_{k+1} = f(x_k, u_k) \f$
    ekfComputeJacobianF(m_x, m_F);               ///< \f$ F \mid_{x = x_k} \f$

    using iDynTree::toEigen;
    auto P(toEigen(m_P));
    auto Phat(toEigen(m_Phat));
    auto F(toEigen(m_F));
    auto Q(toEigen(m_Q));

    // propagate covariance
    Phat = F*P*(F.transpose()) + Q;           ///< \f$ \hat{P}_{k+1} = F_k P_k F_k^T + Q \f$
    m_input_updated = false;
    return true;
}

bool iDynTree::DiscreteExtendedKalmanFilterHelper::ekfUpdate()
{
    if (!m_is_initialized)
    {
        iDynTree::reportError("DiscreteExtendedKalmanFilterHelper", "ekfUpdate", "filter not initialized.");
        return false;
    }

    if (!m_initial_state_set)
    {
        iDynTree::reportError("DiscreteExtendedKalmanFilterHelper", "ekfUpdate", "initial state not set.");
        return false;
    }

    if (!m_initial_state_covariance_set)
    {
        iDynTree::reportError("DiscreteExtendedKalmanFilterHelper", "ekfUpdate", "initial state covariance not set.");
        return false;
    }

    if (!m_measurement_updated)
    {
        iDynTree::reportError("DiscreteExtendedKalmanFilterHelper", "ekfUpdate", "measurements not updated.");
        return false;
    }

    iDynTree::VectorDynSize z;
    z.resize(m_dim_Y);
    ekf_h(m_xhat, z);                             ///< \f$ z_{k+1} = h(\hat{x}_{k+1}) \f$
    ekfComputeJacobianH(m_xhat, m_H);            ///< \f$ H \mid_{x = \hat{x}_{k+1}} \f$

    using iDynTree::toEigen;
    auto P(toEigen(m_P));
    auto Phat(toEigen(m_Phat));
    auto S(toEigen(m_S));
    auto K(toEigen(m_K));
    auto H(toEigen(m_H));
    auto R(toEigen(m_R));
    auto x(toEigen(m_x));
    auto xhat(toEigen(m_xhat));
    auto y(toEigen(m_y) - toEigen(z));        ///< innovation \f$ \tilde{y}_{k+1} = y_{k+1} - z_{k+1} \f$

    S = H*Phat*(H.transpose()) + R;             ///< \f$ S_{k+1} = H_{k+1} \hat{P}_{k+1} H_{k+1}^T + R \f$
    K = Phat*(H.transpose())*(S.inverse());       ///< \f$ K_{k+1} = \hat{P}_{k+1} H_{k+1}^T S_{k+1}^{-1} \f$
    P = Phat - (K*H*Phat);                    ///< \f$ P_{k+1} = \hat{P}_{k+1} - (K_{k+1} H \hat{P}_{k+1}) \f$
    x = xhat + K*y;                           ///< \f$ x_{k+1} = \hat{x}_{k+1} + K_{k+1} \tilde{y}_{k+1} \f$

    m_measurement_updated = false;
    return true;
}

bool iDynTree::DiscreteExtendedKalmanFilterHelper::ekfSetInitialState(const iDynTree::Span< double >& x0)
{
    if ((size_t)x0.size() != m_dim_X)
    {
        iDynTree::reportError("DiscreteExtendedKalmanFilterHelper", "setInitialState", "state size mismatch");
        return false;
    }

    for (int i = 0; i < x0.size(); i++)
    {
        m_x(i) = x0(i);
    }

    m_initial_state_set = true;
    return true;
}

bool iDynTree::DiscreteExtendedKalmanFilterHelper::ekfSetInputVector(const iDynTree::Span< double >& u)
{
    if ((size_t)u.size() != m_dim_U)
    {
        iDynTree::reportError("DiscreteExtendedKalmanFilterHelper", "ekfSetInputVector", "input size mismatch");
        return false;
    }

    for (int i = 0; i < u.size(); i++)
    {
        m_u(i) = u(i);
    }

    m_input_updated = true;
    return true;
}

bool iDynTree::DiscreteExtendedKalmanFilterHelper::ekfSetMeasurementVector(const iDynTree::Span< double >& y)
{
    if ((size_t)y.size() != m_dim_Y)
    {
        iDynTree::reportError("DiscreteExtendedKalmanFilterHelper", "ekfSetMeasurementVector", "measurement size mismatch");
        return false;
    }

    for (int i = 0; i < y.size(); i++)
    {
        m_y(i) = y(i);
    }

    m_measurement_updated = true;
    return true;
}

bool iDynTree::DiscreteExtendedKalmanFilterHelper::ekfSetStateCovariance(const iDynTree::Span< double >& P)
{
    if ((size_t)P.size() != m_dim_X*m_dim_X)
    {
        iDynTree::reportError("DiscreteExtendedKalmanFilterHelper", "setSystemCovariance", "state covariance matrix size mismatch");
        return false;
    }

    m_P = iDynTree::MatrixDynSize(P.data(), m_dim_X, m_dim_X);
    m_initial_state_covariance_set = true;
    return true;
}

bool iDynTree::DiscreteExtendedKalmanFilterHelper::ekfSetSystemNoiseCovariance(const iDynTree::Span< double >& Q)
{
    if ((size_t)Q.size() != m_dim_X*m_dim_X)
    {
        iDynTree::reportError("DiscreteExtendedKalmanFilterHelper", "setSystemNoiseMeanAndCovariance", "noise covariance matrix size mismatch");
        return false;
    }

    m_Q = iDynTree::MatrixDynSize(Q.data(), m_dim_X, m_dim_X);
    return true;
}


bool iDynTree::DiscreteExtendedKalmanFilterHelper::ekfSetMeasurementNoiseCovariance(const iDynTree::Span< double >& R)
{
    if ((size_t)R.size() != m_dim_Y*m_dim_Y)
    {
        iDynTree::reportError("DiscreteExtendedKalmanFilterHelper", "setMeasurementNoiseMeanAndCovariance", "noise covariance matrix size mismatch");
        return false;
    }

    m_R = iDynTree::MatrixDynSize(R.data(), m_dim_Y, m_dim_Y);
    return true;
}

bool iDynTree::DiscreteExtendedKalmanFilterHelper::ekfGetStates(const iDynTree::Span< double >& x) const
{
    if ((size_t)x.size() != m_dim_X)
    {
        iDynTree::reportError("DiscreteExtendedKalmanFilterHelper", "ekfGetStates", "state size mismatch");
        return false;
    }

    for (size_t i = 0; i < m_dim_X; i++)
    {
        x(i) = m_x(i);
    }
    return true;
}

bool iDynTree::DiscreteExtendedKalmanFilterHelper::ekfGetStateCovariance(const iDynTree::Span< double >& P) const
{
    if ((size_t)P.size() != m_P.capacity())
    {
        iDynTree::reportError("DiscreteExtendedKalmanFilterHelper", "ekfGetStateCovariance", "state covariance size mismatch");
        return false;
    }

    for (size_t i = 0; i < m_P.capacity(); i++)
    {
        P(i) = m_P.data()[i];
    }

    return true;
}

