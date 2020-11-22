/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_PRIVATE_UTILS_H
#define IDYNTREE_PRIVATE_UTILS_H


#include <Eigen/Dense>
#include <iDynTree/Core/EigenHelpers.h>

namespace iDynTree
{

    /**
     * Maps a 3d vector to the square of the cross product matrix:
     * v --> (v\times)^2
     * or, if you prefer another notation:
     * v --> S^2(v)
     */
    Eigen::Matrix3d squareCrossProductMatrix(const Eigen::Vector3d & v);

    /**
     * Maps a 3d vector to the cross product matrix:
     * v --> (v\times)
     * or, if you prefer another notation:
     * v --> S(v)
     */
    Eigen::Matrix3d skew(const Eigen::Vector3d & vec);

    /**
     * Efficient version of the copy from one 6D vector to another.
     */
    template <typename vector6d>
    void efficient6dCopy(vector6d* pthis, const vector6d& other)
    {
        toEigen(pthis->getLinearVec3()) = toEigen(other.getLinearVec3());
        toEigen(pthis->getAngularVec3()) = toEigen(other.getAngularVec3());
        return;
    }

    /**
     * Efficient version of the sum of two 6D vectors.
     */
    template <typename vector6d>
    vector6d efficient6dSum(const vector6d & op1, const vector6d & op2)
    {
        vector6d ret;
        toEigen(ret.getLinearVec3()) = toEigen(op1.getLinearVec3()) + toEigen(op2.getLinearVec3());
        toEigen(ret.getAngularVec3()) = toEigen(op1.getAngularVec3()) + toEigen(op2.getAngularVec3());
        return ret;
    }

    /**
     * Efficient version of the different of two 6D vectors.
     */
    template <typename vector6d>
    vector6d efficient6ddifference(const vector6d & op1, const vector6d & op2)
    {
        vector6d ret;
        toEigen(ret.getLinearVec3()) = toEigen(op1.getLinearVec3()) - toEigen(op2.getLinearVec3());
        toEigen(ret.getAngularVec3()) = toEigen(op1.getAngularVec3()) - toEigen(op2.getAngularVec3());
        return ret;
    }

    /**
     * Efficient version of the cross product between a twist
     * and a spatial motion vector (another twist, acceleration, ..)
     */
    template <typename twistType, typename motionVectorType, typename resultType>
    resultType efficientTwistCrossTwist(const twistType & op1, const motionVectorType & op2)
    {
        // res.getLinearVec3()  = this->angularVec3.cross(other.getLinearVec3()) + this->linearVec3.cross(other.getAngularVec3());
        // res.getAngularVec3() =                                                  this->angularVec3.cross(other.getAngularVec3());
        resultType ret;

        toEigen(ret.getLinearVec3()) = toEigen(op1.getAngularVec3()).cross(toEigen(op2.getLinearVec3()))
                                       + toEigen(op1.getLinearVec3()).cross(toEigen(op2.getAngularVec3()));
        toEigen(ret.getAngularVec3()) = toEigen(op1.getAngularVec3()).cross(toEigen(op2.getAngularVec3()));

        return ret;
    }

    /**
     * Efficient version of the cross product between a twist
     * and a spatial force vector (momentum, wrench, ..)
     */
    template <typename twistType, typename momentumVectorType, typename resultType>
    resultType efficientTwistCrossMomentum(const twistType & op1, const momentumVectorType & op2)
    {
        resultType ret;

        toEigen(ret.getLinearVec3())  = toEigen(op1.getAngularVec3()).cross(toEigen(op2.getLinearVec3()));
        toEigen(ret.getAngularVec3()) = toEigen(op1.getLinearVec3()).cross(toEigen(op2.getLinearVec3()))
                                        + toEigen(op1.getAngularVec3()).cross(toEigen(op2.getAngularVec3()));

        return ret;
    }
}


#endif /* IDYNTREE_PRIVATE_UTILS_H */
