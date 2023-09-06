// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_ATTITUDE_ESTIMATOR_UTILS_H
#define IDYNTREE_ATTITUDE_ESTIMATOR_UTILS_H

#include <iDynTree/AttitudeEstimator.h>
#include <iDynTree/EigenHelpers.h>
#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>

/**
 *
 * @brief check a valid measurement
 * @param[in] a vector3
 * @return bool true/false
 */
bool checkValidMeasurement(const iDynTree::Vector3& in, const std::string& measurement_type, bool check_also_zero_vector);

/**
 *
 * @brief get unit vector
 * @param[in] a vector3
 * @return bool false if input vector has zero norm
 */
bool getUnitVector(const iDynTree::Vector3& in, iDynTree::Vector3& out);

/**
 *
 * @brief checks if vector has NaN values
 * any element of vector is NaN implies a NaN vector
 * @param[in] vec vector3
 * @return bool true/false
 */
bool isVectorNaN(const iDynTree::Vector3& vec);

/**
 *
 * @brief checks if vector is a zero vector
 *  all elements of vector are zero implies a zero vector
 * @param[in] vec vector3
 * @return bool true/false
 */
bool isZeroVector(const iDynTree::Vector3& vec);

/**
 *
 * @brief computes the cross vector of two 3D vectors
 * @param[in] a 3D vector
 * @param[in] b 3D vector
 * @return iDynTree::Vector3
 */
iDynTree::Vector3 crossVector(const iDynTree::Vector3& a, const iDynTree::Vector3& b);

/**
 * @brief computes \f$ 3 \times 3 \f$ skew-symmetric matrix (\f$ \mathbb{so}(3) \f$ space) for a given 3d vector (\f$ \mathbb{R}^3 \f$ space)
 *
 * @param[in] omega 3d vector (usually angular velocity)
 * @return iDynTree::Matrix3x3
 */
iDynTree::Matrix3x3 mapR3Toso3(const iDynTree::Vector3& omega);

/**
 * @brief checks if the \f$ 3 \times 3 \f$ matrix is skew-symmetric
 *        \f[ S + S^T = 0 \f]
 * @param[in] S \f$ 3 \times 3 \f$ matrix
 * @return bool true/false
 */
bool checkSkewSymmetricity(const iDynTree::Matrix3x3& S);

/**
 * @brief computes 3D vector (\f$ \mathbb{R}^3 \f$ space) from a skew symmetric matrix (\f$ \mathbb{so}(3) \$ space)
 *
 * @param[in] S \f$ 3 \times 3 \f$ matrix
 * @return iDynTree::Vector3
 */
iDynTree::Vector3 mapso3ToR3(const iDynTree::Matrix3x3& S);

/**
 * @brief computes scalar dot product of two 3-d vectors
 *        Maps  (\f$ \mathbb{R}^n \f$ space) to (\f$ \mathbb{R} \f$ space) through its dual vector
 * @param[in] a 3D vector (not passed as reference, but as a copy to avoid changes in source due to in-place manipulation)
 * @param[in] b 3D vector
 * @return double
 */
double innerProduct(const iDynTree::Vector3 a, const iDynTree::Vector3& b);

/**
 * @brief real part of quaternion,
 *          \f$ s \f$ in \f$ s + i v_1 + i v_2 + i v_3 \f$
 *
 * @param[in] q 4d vector or quaternion
 * @return double
 */
double realPartOfQuaternion(const iDynTree::UnitQuaternion& q);

/**
 * @brief imaginary part of quaternion
 *        \f$ v \f$ in \f$ s + i v_1 + i v_2 + i v_3 \f$
 * @param[in] q 4d vector or quaternion
 * @return iDynTree::Vector3
 */
iDynTree::Vector3 imaginaryPartOfQuaternion(const iDynTree::UnitQuaternion& q);

/**
 * @brief composition operator - quaternion multiplication
 *
 * @param[in] q1 4d vector or quaternion
 * @param[in] q2 4d vector or quaternion
 * @return iDynTree::UnitQuaternion
 */
iDynTree::UnitQuaternion composeQuaternion(const iDynTree::UnitQuaternion& q1, const iDynTree::UnitQuaternion& q2);

/**
 * @brief computes the matrix map of quaternion left multiplication \f$ q1 \circ q2 = q1q2 \f$
 *        in opposition to the right multiplication \f$ q1 \circ q2 = q2q1 \f$
 * @param[in] x 4d vector or quaternion q1
 * @return iDynTree::Matrix4x4
 */
iDynTree::Matrix4x4 mapofYQuaternionToXYQuaternion(const iDynTree::UnitQuaternion &x);


/**
 * @brief composition operator - quaternion multiplication
 *        this method is faster than composeQuaternion()
 * @param[in] q1 4d vector or quaternion
 * @param[in] q2 4d vector or quaternion
 * @return iDynTree::UnitQuaternion
 */
iDynTree::UnitQuaternion composeQuaternion2(const iDynTree::UnitQuaternion &q1, const iDynTree::UnitQuaternion &q2);


/**
 * @brief computes pure quaternion given a 3d vector (uually angular velocity)
 *
 * @param[in] bodyFixedFrameVelocityInInertialFrame 3d vector
 * @return iDynTree::UnitQuaternion
 */
iDynTree::UnitQuaternion pureQuaternion(const iDynTree::Vector3& bodyFixedFrameVelocityInInertialFrame);


/**
 * @brief exponential map for quaternion - maps angular velocities to quaternion
 *
 * \f$ \text{exp}(\omega) = \begin{bmatrix} \text{cos}(\frac{||\omega||}{2}) \\ \text{sin}(\frac{||\omega||}{2})\frac{\omega}{||\omega||}  \end{bmatrix} \f$
 *
 * @param[in] omega angular velocity
 * @return iDynTree::UnitQuaternion
 */
inline iDynTree::UnitQuaternion expQuaternion(iDynTree::Vector3 omega)
{
    iDynTree::UnitQuaternion q;
    q.zero();
    q(0) = 1.0;
    using iDynTree::toEigen;
    double norm{toEigen(omega).norm()};

    if (norm == 0)
    {
        return q;
    }

    double c = std::cos(norm/2);
    double s = std::sin(norm/2)/norm;

    q(0) = c;
    q(1) = omega(0)*s;
    q(2) = omega(1)*s;
    q(3) = omega(2)*s;

    return q;
}

template<class T>
inline bool check_are_almost_equal(const T& x, const T& y, int units_in_last_place)
{
    if (!( std::abs(x- y) <= std::numeric_limits<T>::epsilon()*std::max(std::abs(x), std::abs(y))*units_in_last_place))
    {
        return false;
    }

    return true;
}

#endif

