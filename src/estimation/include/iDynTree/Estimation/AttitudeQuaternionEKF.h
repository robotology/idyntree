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
#include <iDynTree/Core/Direction.h>

namespace iDynTree
{
    const unsigned int output_dimensions_with_magnetometer = 4;        ///< dimension of \f$ \mathbb{R}^3 \times \mathbb{R} \f$ accelerometer measurements and magnetometer yaw measurement
    const unsigned int output_dimensions_without_magnetometer = 3;     ///< dimension of \f$ \mathbb{R}^3 \f$ accelerometer measurements
    const unsigned int input_dimensions = 3;                           ///< dimension of \f$ \mathbb{R}^3 \f$ gyroscope measurements

    /**
     * @struct AttitudeQuaternionEKFParameters Parameters to set up the quaternion EKF
     * @var AttitudeQuaternionEKFParameters::time_step_in_seconds
     * discretization time step in seconds, default value: \f$ 0.01 s \f$
     * @var AttitudeQuaternionEKFParameters::bias_correlation_time_factor
     * time factor modeling how fast the bias can vary, default value: \f$ 0.01 \f$
     * @var AttitudeQuaternionEKFParameters::accelerometer_noise_variance
     * measurement noise covariance \f$ \sigma_{acc}^{2} \f$ of accelerometer measurement, default value: \f$ 0.001 \f$
     * @var AttitudeQuaternionEKFParameters::magnetometer_noise_variance
     * measurement noise covariance \f$ \sigma_{mag}^{2} \f$ of magnetometer measurement, default value: \f$ 0.001 \f$
     * @var AttitudeQuaternionEKFParameters::gyroscope_noise_variance
     * system noise covariance \f$ \sigma_{gyro}^{2} \f$ of gyroscope measurement, since it is the input to the system, default value: \f$ 0.001 \f$
     * @var AttitudeQuaternionEKFParameters::gyro_bias_noise_variance
     * system noise covariance \f$ \sigma_{gyrobias}^{2} \f$ of gyroscope bias estimate, default value: \f$ 0.0001 \f$
     * @var AttitudeQuaternionEKFParameters::initial_orientation_error_variance
     * initial state covariance \f$ \sigma_{q_0}^{2} \f$ of orientation, default value: \f$ 10 \f$
     * @var AttitudeQuaternionEKFParameters::initial_ang_vel_error_variance
     * initial state covariance \f$ \sigma_{\omega_0}^{2} \f$ of angular velocity, default value: \f$ 10 \f$
     * @var AttitudeQuaternionEKFParameters::initial_gyro_bias_error_variance
     * measurement noise covariance \f$ \sigma_{acc}^{2} \f$ of gyroscope bias, default value: \f$ 10 \f$
     * @var AttitudeQuaternionEKFParameters::use_magnetometer_measurements
     * flag to enable the use of magnetometer measurement for yaw correction, default value: false
     */
    struct AttitudeQuaternionEKFParameters {
    double time_step_in_seconds{0.01};
    double bias_correlation_time_factor{0.01};

    // measurement noise - zero mean, and a given variance
    double accelerometer_noise_variance{0.001};
    double magnetometer_noise_variance{0.001};

    // process noise - zero mean, and a given covariance
    double gyroscope_noise_variance{0.001};
    double gyro_bias_noise_variance{0.0001};

    double initial_orientation_error_variance{10};
    double initial_ang_vel_error_variance{10};
    double initial_gyro_bias_error_variance{10};

    bool use_magnetometer_measurements{false};
    };

    /**
     * @class AttitudeQuaternionEKF implements a Quaternion based Discrete Extended Kalman Filter fusing IMU measurements,
     * to give estimates of orientation, angular velocity and gyroscope bias
     *
     * It follows the implementation detailed in
     * <a href="https://wuecampus2.uni-wuerzburg.de/moodle/pluginfile.php/1109745/mod_resource/content/1/QEKF_Floatsat_WS16.pdf">Quaternion Based Extended Kalman Filter, slides by Michael Stohmeier</a>
     * The filter is used to estimate the states \f$ X = \begin{bmatrix} {^A}q_B \\ {^B}\Omega_{A,B} \\ {^B}b \end{bmatrix}^T \f$
     * where \f$ {^A}q_B \in \mathbb{R}^4 \f$  is the quaternion representing the orientation of a body(IMU) frame with respect to an inertial frame ,
     *       \f$ {^B}\Omega_{A,B} \in \mathbb{R}^3 \f$  is the angular velocity of a body(IMU) frame with respect to an inertial frame, expressed in the body frame and
     *       \f$ {^B}b \in \mathbb{R}^3 \f$  is the gyroscope bias expressed in the body frame.
     * @note: we will drop the subscripts and superscripts in the rest of the documentation for convenience
     *
     * Discretized dynamics during the prediction step,
     * \f[ \hat{{x}}_{k+1} = \begin{bmatrix} q_{k} \otimes \text{exp}(\omega \Delta T) \\ y_{gyro_{k}} - b_{k} \\ (1 - \lambda_{b} \Delta t)b_k \end{bmatrix} \f]
     *
     * Measurement model for accelerometer is given as,
     * \f[ h_{acc}(\hat{x}_{k+1}) = \begin{bmatrix} 2(q_1q_3 - q_0q_2) \\ 2(q_2q_3 - q_0q_1) \\ q_0^2 - q_1^2 - q_2^2 + q_3^2 \end{bmatrix} \f]
     * obtained from \f$ {^w}R_b^T e_3 \f$ of the assumed gravity direction.
     *
     * Measurement model for magnetometer measurement is given as,
     * \f[ h_{mag}(\hat{x}_{k+1}) = atan2( 2(q_0q_3 + q_1q_2),1 - 2(q_2^2 + q_3^2) ) \f]
     *
     * The linearized system propogation and measurement model is obtained by computing Jacobins F and H with respect to the state.
     *
     * The zero mean, additive Gaussian noise can be set using the covariance matrices which will be used during predict and update steps.
     *
     * The propagateStates() method is called to set the input vector for the EKF, then ekfPredict() is called to propagate the state through the propagation function f()
     * and propate the state covariance using the Jacobian F.
     *
     * The updateFilterWithMeasurements() is called to set the measurement vector for the EKF, and then ekfUpdate is used to correct the state estimate and its covariance
     * using the measurement model function h() and the measurement Jacobian H.
     *
     * The usage of the QEKF should follow the decribed procedure below,
     * - instantiate the filter
     * - set parameters
     * - call initializeFilter() (this is necessary for resizing the buffers, the user should call this method after setting parameters)
     * - use setInternalState() to set initial state (The filter will throw an error, if this is not called atleast once, this enforces the user to set intial state)
     * - Once initialized, the following filter methods can be run in a loop to get the orientation estimates,
     *     - propagateStates() method to propagate the states and covariance
     *     - updateFilterWithMeasurements() method to correct the predicted states and covariance
     *     - getInternalState() or getOrientationEstimate*() methods to get the entire state estimate or only the attitude estimated in desired representation
     *
     * @warning calling the method useMagnetometerMeasurements() while the estimator is running, will reset the filter, reinitialize the filter to resize buffers
     * and sets the previous estiamted state as the inital state.
     * @note calling other set parameter methods does not reset the filter, since they are not associated with changing the output dimensions
     *
     */
    class AttitudeQuaternionEKF : public IAttitudeEstimator,
                                  public DiscreteExtendedKalmanFilterHelper
    {
    public:
        AttitudeQuaternionEKF();

        /**
         * @brief Get filter parameters as a struct.
         * @param[out] params object of AttitudeQuaternionEKFParameters passed as reference
         */
        void getParameters(AttitudeQuaternionEKFParameters& params) {params = m_params_qekf;}

        /**
          * @brief Set filter parameters with the struct members.
          * This resets filter since it also calls useMagnetometerMeasurements(flag)
          * (if the use_magnetometer_measurements flag has been toggled).
          * @param[in] params object of AttitudeQuaternionEKFParameters passed as a const reference
          * @return true/false if successful/not
          */
        void setParameters(const AttitudeQuaternionEKFParameters& params)
        {
            m_params_qekf = params;
            useMagnetometerMeasurements(params.use_magnetometer_measurements);
        }

        /**
         * @brief Set the gravity direction assumed by the filter
         * This affects the measurement model function h() and Jacobian H
         * @param[in] gravity_dir gravity direction
         */
        void setGravityDirection(const iDynTree::Direction& gravity_dir);

        /**
         * @brief set discretization time step in seconds
         * @param[in] time_step_in_seconds time step
         */
        void setTimeStepInSeconds(double time_step_in_seconds) {m_params_qekf.time_step_in_seconds = time_step_in_seconds; }

        /**
         * @brief set bias correlation time factor
         * @param[in] bias_correlation_time_factor time factor for bias evolution
         */
        void setBiasCorrelationTimeFactor(double bias_correlation_time_factor) { m_params_qekf.bias_correlation_time_factor = bias_correlation_time_factor; }

        /**
         * @brief set flag to use magnetometer measurements
         * @param[in] use_magnetometer_measurements enable/disable magnetometer measurements
         * @note calling this method with the flag same as current flag value will not change anything,
         *  meanwhile a new flag setting will reset the filter, reinitialize the filter and
         *  set the previous state as filter's initial state and previous state covariance as filter's intial state covariance
         */
        bool useMagnetometerMeasurements(bool use_magnetometer_measurements);

        /**
         * @brief prepares the measurement noise covariance matrix and calls ekfSetMeasurementNoiseMeanAndCovariance()
         * measurement noise depends only on accelerometer xyz (and magnetometer z)
         * @note the noise has zero mean (basically passes a zero vector with covariance matrix)
         * @param[in] acc variance for accelerometer measurements
         * @param[in] mag variance for magnetometer measurements
         * @return true/false if successful/not
         */
        bool setMeasurementNoiseVariance(double acc, double mag);

        /**
         * @brief prepares the system noise covariance matrix and calls ekfSetSystemNoiseMeanAndCovariance()
         * process noise depends on gyro measurement and gyro bias estimate - since gyro measurement is passed as input
         * @note the noise has zero mean (basically passes a zero vector with covariance matrix)
         * measurement noise depends only on accelerometer xyz (and magnetometer z)
         * @param[in] gyro variance for gyroscope measurements
         * @param[in] gyro_bias variance for gyroscope bias estimates
         * @return true/false if successful/not
         */
        bool setSystemNoiseVariance(double gyro, double gyro_bias);

        /**
         * @brief prepares the state covariance matrix and calls ekfSetStateCovariance()
         * @param[in] orientation_var variance for intial orientation state estimate
         * @param[in] ang_vel_var variance for initial angular velocity state estimate
         * @param[in] gyro_bias_var variance for intial gyro bias state estimate
         * @return true/false if successful/not
         */
        bool setInitialStateCovariance(double orientation_var, double ang_vel_var, double gyro_bias_var);

        /**
         * @brief intializes the filter by resizing buffers and setting parameters
         * - sets state, output and input dimensions for the ekf
         * - resizes internal buffers
         * - calls ekfInit()
         * - sets system noise, measurement noise and initial state covariance
         * - if successful sets initialized flag to true
         * @return true/false if successful/not
         */
        bool initializeFilter();


        bool updateFilterWithMeasurements(const iDynTree::LinearAccelerometerMeasurements& linAccMeas,
                                          const iDynTree::GyroscopeMeasurements& gyroMeas) override;
        bool updateFilterWithMeasurements(const iDynTree::LinearAccelerometerMeasurements& linAccMeas,
                                          const iDynTree::GyroscopeMeasurements& gyroMeas,
                                          const iDynTree::MagnetometerMeasurements& magMeas) override;
        bool propagateStates() override;
        bool getOrientationEstimateAsRotationMatrix(iDynTree::Rotation& rot) override;
        bool getOrientationEstimateAsQuaternion(iDynTree::UnitQuaternion& q) override;
        bool getOrientationEstimateAsRPY(iDynTree::RPY& rpy) override;
        size_t getInternalStateSize() const override;
        bool getInternalState(const iDynTree::Span<double> & stateBuffer) const override;
        bool getDefaultInternalInitialState(const iDynTree::Span<double> & stateBuffer) const override;
        bool setInternalState(const iDynTree::Span<double> & stateBuffer) override;
        bool setInternalStateInitialOrientation(const iDynTree::Span<double>& orientationBuffer) override;

    protected:
        AttitudeEstimatorState m_state_qekf, m_initial_state_qekf;
        AttitudeQuaternionEKFParameters m_params_qekf;   ///< struct holding the QEKF parameters


    private:
        /**
         * discrete system propagation \f$ f(X, u) = f(X, y_gyro) \f$
         * where \f$ X = \begin{bmatrix} q_0 &  q_1 & q_2 & q_3 & \omega_x & \omega_y & \omega_z & \b_x & \b_y & \b_z \end{bmatrix}^T \f$
         * \f$ u = \begin{bmatrix} y_{gyro}_x & y_{gyro}_y & y_{gyro}_z \end{bmatrix}^T \f$
         * \f$ f(X, u) = \begin{bmatrix} q_{k} \otimes \text{exp}(\omega \Delta T) \\ y_{gyro} - b \\ (1 - \lambda_{b} \Delta t)b \end{bmatrix}\f$
         */
        bool ekf_f(const iDynTree::VectorDynSize& x_k,
               const iDynTree::VectorDynSize& u_k,
               iDynTree::VectorDynSize& xhat_k_plus_one) override;

        /**
         * discrete measurement prediction
         * where \f$ h(X) = \begin{bmatrix} h_{acc}(X) & h_{mag}(X) \end{bmatrix}^T \f$
         * \f$ h_{acc}(X) = R^T \begin{bmatrix} 0 \\  0 \\ -1 \end{bmatrix} \f$
         * \f$ h_{mag}(X) = atan2(tan(yaw))\f$
         */
        bool ekf_h(const iDynTree::VectorDynSize& xhat_k_plus_one,
               iDynTree::VectorDynSize& zhat_k_plus_one) override;

        /**
         * @brief Describes the system Jacobian necessary for the propagation of predicted state covariance
         *        The analytical Jacobian describing the partial derivative of the system propagation with respect to the state
         * @param[in] x system state
         * @param[out] F system Jacobian
         * @return bool true/false if successful or not
         */
        bool ekfComputeJacobianF(iDynTree::VectorDynSize& x, iDynTree::MatrixDynSize& F) override;
        bool ekfComputeJacobianF(iDynTree::VectorDynSize& x, iDynTree::VectorDynSize& u, iDynTree::MatrixDynSize& F) override;

        /**
         * @brief Describes the measurement Jacobian necessary for computing Kalman gain and updating the predicted state and its covariance
         *        The analytical Jacobian describing the partial derivative of the measurement model with respect to the state
         * @param[in] x system state
         * @param[out] H measurement Jacobian
         * @return bool true/false if successful or not
         */
        bool ekfComputeJacobianH(iDynTree::VectorDynSize& x, iDynTree::MatrixDynSize& H) override;

        /** @brief prepares the system noise covariance matrix using internal struct params
         * system  model is as good as gyroscope measurement and bias estimate
         * system noise covariance can be descibed as \f$ Q = F_u  U  {F_u}^T \f$
         * where \f$ F = \begin{bmatrix} \frac{\partial f}{\partial y_gyro} & \frac{\partial f}{ \partial x_gyrobias} \end{bmatrix} \f$
         * \f$ = \begin{bmatrix} 0_{4 \times 3} & 0_{4 \times 3} \\ I_{3 \times 3} & 0_{3 \times 3} \\ 0_{3 \times 3} & I_{3 \times 3} \end{bmatrix}\f$
         * \f$ U = diag(\begin{bmatrix} \sigma_{gyro}^{2} I_{3 \times 3} & \sigma_{gyrobias}^{2} I_{3 \times 3} \end{bmatrix}) \f$
         * @param[in] Q matrix container as reference
         */
        void prepareSystemNoiseCovarianceMatrix(iDynTree::MatrixDynSize &Q);

        /** @brief prepares the measurement noise covariance matrix using internal struct parameters
         * measurement noise depends only on accelerometer measurement along x-,y- and z- directions
         * along with magnetometer z-direction if included
         * measurement noise covariance can be descibed as,
         * \f$ R = \begin{bmatrix} \sigma_{acc}^{2} I_{3 \times 3} & 0_{3 \times 1} \\ 0_{1 \times 3} & \sigma_{mag}^{2}\f$
         * if magnetometer measurements is also considered. In case of magnetometer measurements not being considered, it is reduced
         * to the \f$ 3 \times 3 \f$ matrix
         * @param[in] R matrix container as reference
         */
        void prepareMeasurementNoiseCovarianceMatrix(iDynTree::MatrixDynSize &R);

        /**
         * @brief serializes the state struct to state x of VectorDynSize
         */
        void serializeStateVector();

        /**
         * @brief deserializes state x of VectorDynSize to the state struct
         */
        void deserializeStateVector();

        /**
         * @brief serializes the accelerometer and magenetometer measurements into y vector
         * since DiscreteExtendedKalmanFilter expects a VectorDynSize including all necessary measurements
         */
        void serializeMeasurementVector();


        /**
         * @brief serializes measurements, calls ekfUpdate step and
         * gets state estimate corrected by measurements
         *
         * @return true/false, if successful/not
         */
        bool callEkfUpdate();

        iDynTree::Rotation m_orientationInSO3;                   ///< orientation estimate as rotation matrix \f$ {^A}R_B \f$ where \f$ A \f$ is inertial frame and \f$ B \f$ is the frame attached to the body
        iDynTree::RPY m_orientationInRPY;                        ///< orientation estimate as a 3D vector in RPY representation, where \f$ {^A}R_B = Rot_z(yaw)Rot_y(pitch)Rot_x(roll) \f$

        iDynTree::GyroscopeMeasurements m_Omega_y;               ///< 3d gyroscope measurement giving angular velocity of body wrt inertial frame, expressed in body frame
        iDynTree::LinearAccelerometerMeasurements m_Acc_y;       ///< 3d accelerometer measurement giving proper classical acceleration expressed in body frame
        double m_Mag_y;                                          ///< magnetometer yaw measurement expressed in body frame

        iDynTree::VectorDynSize m_x;                             ///< state vector for the EKF - orientation, angular velocity, gyro bias
        iDynTree::VectorDynSize m_y;                             ///< measurement vector for the EKF - accelerometer (and magnetometer yaw)
        iDynTree::VectorDynSize m_u;                             ///< input vector for the EKF - gyroscope measurement

        size_t m_state_size;                                     ///< state dimensions
        size_t m_output_size;                                    ///< output dimensions
        size_t m_input_size;                                     ///< input dimensions
        bool m_initialized{false};                               ///< flag to check if QEKF is initialized

        iDynTree::Matrix4x4 m_Id4;                               ///< \f$ 4 \times 4 \f$  identity matrix
        iDynTree::Matrix3x3 m_Id3;                               ///< \f$ 3 \times 3 \f$  identity matrix
        iDynTree::Direction m_gravity_direction;                 ///< direction of the gravity vector expressed in the inertial frame denoted by \f$ A \f$, default set to \f$ e_3 = \begin{bmatrix} 0 & 0 & 1.0 \end{bmatrix}^T \f$
    };

}

#endif
