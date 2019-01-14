/*
 * Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_ATTITUDE_ESTIMATOR_UTILS_H
#define IDYNTREE_ATTITUDE_ESTIMATOR_UTILS_H

#include <iDynTree/Estimation/AttitudeEstimator.h>
#include <iDynTree/Core/EigenHelpers.h>


/**
 *
 * @brief computes the cross vector of two 3D vectors
 * @param[in] a 3D vector
 * @param[in] b 3D vector
 * @return iDynTree::Vector3
 */
iDynTree::Vector3 crossVector(const iDynTree::Vector3& a, const iDynTree::Vector3& b);

/**
 * @brief computes \f$ 3 \times 3 \f$ skew-symmetric matrix (\f$ \mathbb{so}(3) \$ space) for a given 3d vector (\f$ \mathbb{R}^3 \f$ space)
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
double realPartOfQuaternion(const iDynTree::Quaternion& q);

/**
 * @brief imaginary part of quaternion
 *        \f$ v \f$ in \f$ s + i v_1 + i v_2 + i v_3 \f$
 * @param[in] q 4d vector or quaternion
 * @return iDynTree::Vector3
 */
iDynTree::Vector3 imaginaryPartOfQuaternion(const iDynTree::Quaternion &q);

/**
 * @brief composition operator - quaternion multiplication
 *
 * @param[in] q1 4d vector or quaternion
 * @param[in] q2 4d vector or quaternion
 * @return iDynTree::Quaternion
 */
iDynTree::Quaternion composeQuaternion(const iDynTree::Quaternion &q1, const iDynTree::Quaternion &q2);

/**
 * @brief computes the matrix map of quaternion left multiplication q1q2
 *
 * @param[in] x 4d vector or quaternion q1
 * @return iDynTree::Matrix4x4
 */
iDynTree::Matrix4x4 mapofYQuaternionToXYQuaternion(const iDynTree::Quaternion &x);


/**
 * @brief composition operator - quaternion multiplication
 *        this method is faster than composeQuaternion()
 * @param[in] q1 4d vector or quaternion
 * @param[in] q2 4d vector or quaternion
 * @return iDynTree::Quaternion
 */
iDynTree::Quaternion composeQuaternion2(const iDynTree::Quaternion &q1, const iDynTree::Quaternion &q2);


/**
 * @brief computes pure quaternion given a 3d vector (uually angular velocity)
 *
 * @param[in] bodyFixedFrameVelocityInInertialFrame 3d vector
 * @return iDynTree::Quaternion
 */
iDynTree::Quaternion pureQuaternion(const iDynTree::Vector3& bodyFixedFrameVelocityInInertialFrame);

/**
 * @brief get RPY euler angles from quaternion
 *
 * First rotate about x (roll), then y(pitch) and z(yaw)
 * characterized by the matrix multiplication
 * R = Rz*Ry*Rx
 *
 * @param[in] q quaternion
 * @return iDynTree::RPY
 */
iDynTree::RPY quaternion2eulerRPY(const iDynTree::Quaternion &q);

#endif