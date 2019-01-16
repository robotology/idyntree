/*
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef ATTITUDE_QUATERNION_EKF_H
#define ATTITUDE_QUATERNION_EKF_H

#include <iDynTree/Estimation/AttitudeEstimator.h>
#include <iDynTree/Estimation/ExtendedKalmanFilter.h>
#include <iDynTree/Estimation/AttitudeEstimatorUtils.h>

namespace iDynTree
{
    const unsigned int output_dimensions_with_magnetometer = 4;
    const unsigned int output_dimensions_without_magnetometer = 3;
    const unsigned int input_dimensions = 3;

    struct AttitudeQuaternionEKFParameters {
    //TODO: add default values and docs
    double time_step_in_seconds{0.01};
    double bias_correlation_time_factor{0.01};

    // measurement noise - zero mean, and a given variance
    double accelerometer_noise_variance{0.001};
    double magnetometer_noise_variance{0.001};

    // process noise - zero mean, and a given covariance
    double gyroscope_noise_variance{0.001};
    double gyro_bias_noise_variance{0.0001};

    double initial_orientation_error_variance{100};
    double initial_ang_vel_error_variance{100};
    double initial_gyro_bias_error_variance{100};

    bool use_magenetometer_measurements{false};
    };

    class AttitudeQuaternionEKF : public IAttitudeEstimator,
                                  public DiscreteExtendedKalmanFilter
    {
    public:
        AttitudeQuaternionEKF();

        void getParams(AttitudeQuaternionEKFParameters& params) {params = m_params;}
        void setParams(const AttitudeQuaternionEKFParameters& params) {m_params = params;}

        void setTimeStepInSeconds(double time_step_in_seconds) {m_params.time_step_in_seconds = time_step_in_seconds; }
        void setBiasCorrelationTimeFactor(double bias_correlation_time_factor) { m_params.bias_correlation_time_factor = bias_correlation_time_factor; }
        bool useMagnetometerMeasurements(bool use_magenetometer_measurements);

        // measurement noise depends only on accelerometer xyz and magnetometer z
        bool setMeasurementNoiseVariance(double acc, double mag);
        // process noise depends on gyro measurement and gyro bias estimate - since gyro measurement is passed as input
        bool setSystemNoiseVariance(double gyro, double gyro_bias);
        bool setInitialStateCovariance(double orientation_var, double ang_vel_var, double gyro_bias_var);

        bool initializeFilter();

        bool updateFilterWithMeasurements(const iDynTree::LinearAccelerometerMeasurements& linAccMeas,
                                        const iDynTree::GyroscopeMeasurements& gyroMeas) override;


        bool updateFilterWithMeasurements(const iDynTree::LinearAccelerometerMeasurements& linAccMeas,
                                                const iDynTree::GyroscopeMeasurements& gyroMeas,
                                                const iDynTree::MagnetometerMeasurements& magMeas) override;

        bool propagateStates() override;

        bool getOrientationEstimateAsRotationMatrix(iDynTree::Rotation& rot) override;

        bool getOrientationEstimateAsQuaternion(iDynTree::Quaternion& q) override;

        bool getOrientationEstimateAsRPY(iDynTree::RPY& rpy) override;

        // State
        size_t getInternalStateSize() const override;
        bool getInternalState(iDynTree::Span<double> & stateBuffer) const override;
        bool getInternalInitialState(iDynTree::Span<double> & stateBuffer) const override;
        bool setInternalState(iDynTree::Span<double> & stateBuffer) override;

    private:
        /**
         * discrete system propagation \f$ f(X, u) = f(X, y_gyro) \f$
         * where \f$ X = \begin{bmatrix} q_0 &  q_1 & q_2 & q_3 & \omega_x & \omega_y & \omega_z & \b_x & \b_y & \b_z \end{bmatrix}^T \f$
         * \f$ u = \begin{bmatrix} y_{gyro}_x & y_{gyro}_y & y_{gyro}_z \end{bmatrix}^T \f$
         * \f$ = \begin{bmatrix} q + \Delta t. \frac{1}{2}q \circ \begin{bmatrix} 0 \\ \omega^b \end{bmatrix} \\ y_{gyro} - b \\ (1 - \lambda_{b} \Delta t)b \end{bmatrix}\f$
         */
        bool f(const iDynTree::VectorDynSize& x_k,
               const iDynTree::VectorDynSize& u_k,
               const iDynTree::VectorDynSize& w_k,
               iDynTree::VectorDynSize& xhat_k_plus_one) override;

        /**
         * discrete measurement prediction
         * where \f$ h(X) = \begin{bmatrix} h_{acc}(X) & h_{mag}(X) \end{bmatrix}^T \f$
         * \f$ h_{acc}(X) = R^T \begin{bmatrix} 0 \\  0 \\ -1 \end{bmatrix} \f$
         * \f$ h_{mag}(X) = atan2(tan(yaw))\f$
         */
        bool h(const iDynTree::VectorDynSize& xhat_k_plus_one,
               const iDynTree::VectorDynSize& v_k_plus_one,
               iDynTree::VectorDynSize& zhat_k_plus_one) override;


        bool computejacobianF(iDynTree::VectorDynSize& x, iDynTree::MatrixDynSize& F) override;
        bool computejacobianH(iDynTree::VectorDynSize& x, iDynTree::MatrixDynSize& H) override;

        /**
         * system  model is as good as gyroscope measurement and bias estimate
         * system noise covariance can be descibed as \f$ Q = F_u  U  {F_u}^T \f$
         * where \f$ F = \begin{bmatrix} \frac{\partial f}{\partial y_gyro} & \frac{\partial f}{ \partial x_gyrobias} \end{bmatrix} \f$
         * \f$ = \begin{bmatrix} 0_{4 \times 3} & 0_{4 \times 3} \\ I_{3 \times 3} & 0_{3 \times 3} \\ 0_{3 \times 3} & I_{3 \times 3} \end{bmatrix}\f$
         * \f$ U = diag(\begin{bmatrix} \sigma_{gyro}^{2} I_{3 \times 3} & \sigma_{gyrobias}^{2} I_{3 \times 3} \end{bmatrix}) \f$
         */
        void prepareSystemNoiseCovarianceMatrix(iDynTree::MatrixDynSize &Q);

        /**
         * measurement noise depends only on accelerometer measurement along x-,y- and z- directions
         * along with magnetometer z-direction if included
         * measurement noise covariance can be descibed as \f$ Q = F_u  U  {F_u}^T \f$
         * \f$ R = \begin{bmatrix} \sigma_{acc}^{2} I_{3 \times 3} & 0_{3 \times 1} \\ 0_{1 \times 3} & \sigma_{mag}^{2}\f$
         */
        void prepareMeasurementNoiseCovarianceMatrix(iDynTree::MatrixDynSize &R);

        void serializeStateVector();
        void deserializeStateVector();
        void serializeMeasurementVector();

        bool callEkfUpdate();

        AttitudeQuaternionEKFParameters m_params;
        struct {
            iDynTree::Quaternion m_orientation;
            iDynTree::Vector3 m_angular_velocity;
            iDynTree::Vector3 m_gyroscope_bias;
        } m_state, m_initial_state;

        iDynTree::Rotation m_orientationInSO3;
        iDynTree::RPY m_orientationInRPY;

        iDynTree::GyroscopeMeasurements m_Omega_y;
        iDynTree::LinearAccelerometerMeasurements m_Acc_y;
        double m_Mag_y;

        iDynTree::VectorDynSize m_x;
        iDynTree::VectorDynSize m_y;
        iDynTree::VectorDynSize m_u;
        iDynTree::VectorDynSize m_v;
        iDynTree::VectorDynSize m_w;

        size_t m_state_size;
        size_t m_output_size;
        size_t m_input_size;
        bool m_initialized{false};

        iDynTree::Matrix4x4 m_Id4;
        iDynTree::Matrix3x3 m_Id3;
    };

}

#endif