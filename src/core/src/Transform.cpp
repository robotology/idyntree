/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/Rotation.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/Wrench.h>
#include <iDynTree/Core/SpatialMomentum.h>
#include <iDynTree/Core/SpatialAcc.h>
#include <iDynTree/Core/SpatialInertia.h>
#include <iDynTree/Core/Direction.h>
#include <iDynTree/Core/Axis.h>
#include <iDynTree/Core/ArticulatedBodyInertia.h>
#include <iDynTree/Core/MatrixFixSize.h>

#include <iDynTree/Core/PrivateUtils.h>
#include <iDynTree/Core/Utils.h>

#include <Eigen/Dense>

#include <iostream>
#include <sstream>

#include <cassert>

typedef Eigen::Matrix<double,3,3,Eigen::RowMajor> Matrix3dRowMajor;

namespace iDynTree
{

/**
 * Static functions
 *
 * We define here templated functions implementing geometrical operations, listed below
 * (with the respective instanciation result):
 *
 * - geometric operations on 3x1 vectors (positions and rotations and homogemeous tranform)
 *
 *   static Position transform(const Transform & op1, const Position & op2)
 *
 * - geometric operations on spatial 6x1 vectors (Twist, SpatialAcc, SpatialMomentum, Wrench).
 *   The distinction between these vectors is handled in Position and Rotation operations
 *   through overloaded functions.
 *
 *   static Wrench   transform(const Transform & op1, const Wrench   & op2)
 *   static Twist    transform(const Transform & op1, const Twist    & op2)
 *   static SpatialMomentum transform(const Transform & op1, const SpatialMomentum & op2)
 *   static SpatialAcc      transform(const Transform & op1, const SpatialAcc & op2)
 *
 * - geometric operations on spatial 6x6 matrices (SpatialInertia).
 *
 *   static SpatialInertia  transform(const Transform & op1, const SpatialInertia & op2)
 */

template<class T>
static T transform(const Transform & op1, const T & op2);

template<>
Position transform(const Transform& op1, const Position& op2);

template<>
SpatialInertia transform(const Transform& op1, const SpatialInertia& op2);

template<>
ArticulatedBodyInertia transform(const Transform& op1, const ArticulatedBodyInertia& op2);

template<typename spatialForceType>
spatialForceType transformWrenchEfficient(const Transform & op1, const spatialForceType & op2);

template<typename spatialVelType>
spatialVelType transformTwistEfficient(const Transform & op1, const spatialVelType & op2);

/**
 * Class functions
 */

Transform::Transform(): pos(),
                        rot()
{
}

Transform::Transform(const Rotation& _rot, const Position& origin): pos(origin),
                                                                    rot(_rot)
{
}

Transform::Transform(const Matrix4x4& transform)
{
    fromHomogeneousTransform(transform);
}

Transform::Transform(const Transform& other): pos(other.getPosition()),
                                              rot(other.getRotation())
{
}

Transform& Transform::operator=(const Transform& other)
{
    if (this == &other) return *this;
    this->pos = other.getPosition();
    this->rot = other.getRotation();
    return *this;
}

const Position& Transform::getPosition() const
{
    return this->pos;
}

const Rotation& Transform::getRotation() const
{
    return this->rot;
}

void Transform::setPosition(const Position& position)
{
    // set position
    this->pos = position;
}

void Transform::setRotation(const Rotation& rotation)
{
    // set rotation
    this->rot = rotation;
}


Transform Transform::compose(const Transform& op1, const Transform& op2)
{
    return Transform(op1.getRotation()*op2.getRotation(),op1.getRotation()*op2.getPosition()+op1.getPosition());
}

Transform Transform::inverse2(const Transform& trans)
{
    Transform result;

    result.setRotation(trans.getRotation().inverse());
    result.setPosition(-(result.getRotation()*trans.getPosition()));

    return result;
}

Transform Transform::operator*(const Transform& other) const
{
    return compose(*this,other);
}

Transform Transform::inverse() const
{
    return Transform::inverse2(*this);
}


Position Transform::operator*(const Position& op2) const
{
    return transform<Position>(*this,op2);
}

Twist Transform::operator*(const Twist& op2) const
{
    return transformTwistEfficient(*this,op2);
}

SpatialForceVector Transform::operator*(const SpatialForceVector& op2) const
{
    return transformWrenchEfficient(*this,op2);
}

Wrench Transform::operator*(const Wrench& op2) const
{
    return transformWrenchEfficient(*this,op2);
}

SpatialMomentum Transform::operator*(const SpatialMomentum& op2) const
{
    return transformWrenchEfficient(*this,op2);
}

SpatialAcc Transform::operator*(const SpatialAcc& op2) const
{
    return transformTwistEfficient(*this,op2);
}

SpatialMotionVector Transform::operator*(const SpatialMotionVector& op2) const
{
    return transformTwistEfficient(*this,op2);
}


SpatialInertia Transform::operator*(const SpatialInertia& op2) const
{
    return transform<SpatialInertia>(*this,op2);
}

ArticulatedBodyInertia Transform::operator*(const ArticulatedBodyInertia& other) const
{
    return transform<ArticulatedBodyInertia>(*this,other);
}

Direction Transform::operator*(const Direction& op2) const
{
    return this->getRotation()*op2;
}

Axis Transform::operator*(const Axis& op2) const
{
    return Axis(this->getRotation()*op2.getDirection(),(*this)*op2.getOrigin());
}

void Transform::fromHomogeneousTransform(const Matrix4x4& transform)
{
    Eigen::Map<Eigen::Matrix<double,4,4,Eigen::RowMajor> > homogeneousMatrix((double*) transform.data());

    toEigen(pos) = homogeneousMatrix.block<3, 1>(0, 3);
    toEigen(rot) = homogeneousMatrix.block<3, 3>(0, 0);
}

Transform Transform::Identity()
{
    return Transform(Rotation::Identity(),Position::Zero());
}

std::string Transform::toString() const
{
    std::stringstream ss;

    ss << rot.toString() << " "
       << pos.toString();

    ss << std::endl;

    return ss.str();
}

std::string Transform::reservedToString() const
{
    return this->toString();
}


/**
 * Static functions
 *
 *
 */
    template<class T>
    T transform(const Transform& op1, const T& op2)
    {
        /*
         *### Twist Transform::transform(const Transform& op1, const Twist& op2)
         *
         * The Twist is a Spatial Motion vector.
         * We decompose the spatial transform as the product of the translation
         * and the rotation:
         * B^X_A = |R   0|*|1   0|
         *         |0   R| |p˜' 1|
         *
         * A^X_B = |1   0|*|R'  0 | = xlt(-p) * rot(R')
         *         |p˜  1| |0   R'|
         * where R is the rotation transforming coordinates from [A] to [B],
         * R' is the rotation transforming coordinates from [B] to [A] (op1.rot),
         * p the position of B with respect to A (\vec{AB} expressed in Frame A coordinates, op1.pos)
         * xlt(-p) the spatial translation component of A^X_B = xlt(-op1.pos)
         * rot(R') the spatial rotation component of A^X_B = rot(op1.rot)
         *
         * (For more details, refer to Featherstone's Rigid Body Dynamics Algorithms, 2.8 Coordinate Transforms)
         *
         * so, considering a given "twist":
         * A^X_B * twist = xlt(-op1.pos) * rot(op1.rot) * twist
         *
         * If we associate (xlt(-op1.pos)*) to the operator Position::operator * (Twist&),
         * and (rot(op1.rot)*) to the operator Rotation::operator * (Twist&), we then get:
         *
         * op1 * twist = op1.pos * (op1.rot * op2)
         *
         *### Wrench Transform::transform(const Transform& op1, const Wrench& op2)
         *
         * Same thing as for Twist, but with translation and rotation for a Spatial Force vector:
         *
         * A^X_B^* = xlt(p)' * rot(R')
         *
         * Position::operator * (Twist&) |-> (xlt(p)' *)
         * Rotation::operator * (Twist&) |-> (rot(R') *)
         *
         *### SpatialMomentum Transform::transform(const Transform& op1, const SpatialMomentum& op2)
         *
         * The Spatial Momentum is a Spatial Force vector
         *
         *###SpatialAcc Transform::transform(const Transform& op1, const SpatialAcc& op2)
         *
         * The Spatial Acceleration is a Spatial Motion vector
         *
         */

        return op1.getPosition()*(op1.getRotation()*op2);
    }

    // \todo TODO have a unique mySkew
    template<class Derived>
    inline Eigen::Matrix<typename Derived::Scalar, 3, 3> mySkew(const Eigen::MatrixBase<Derived> & vec)
    {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
        return (Eigen::Matrix<typename Derived::Scalar, 3, 3>() << 0.0, -vec[2], vec[1], vec[2], 0.0, -vec[0], -vec[1], vec[0], 0.0).finished();
    }

    template<>
    Position transform(const Transform& op1, const Position& op2)
    {
        /* op2 is a vector of dimention 3. We expand here the dot product
         * between the homogeneous transform and the homogeneous position
         * (vector of dim 4), as being : [T].[x y z 1]'
         */
        return (op1.getRotation()*op2+op1.getPosition());
    }

    template<>
    SpatialInertia transform(const Transform& op1, const SpatialInertia& op2)
    {
        /* The transform of a Spatial Inertia is defined as follows:
         * A^I = A^X_B^* * B^I * B^X_A
         */

        // mass clearly remains the same
        double newMass = op2.getMass();

        // the com is transformed as any position
        Position newCenterOfMass = transform<Position>(op1,Position(op2.getCenterOfMass()));

        // the rotational inertial is rotated and then
        // the parallel axis theorem applies
        RotationalInertiaRaw newRotInertia;
        Eigen::Map<Eigen::Matrix3d> newI(newRotInertia.data());
        RotationalInertiaRaw oldRotInertiaWrtCom = op2.getRotationalInertiaWrtCenterOfMass();
        Eigen::Map<const Eigen::Matrix3d> oldIWrtCom(oldRotInertiaWrtCom.data());

        Eigen::Map<const Matrix3dRowMajor> R(op1.getRotation().data());
        Eigen::Map<const Eigen::Vector3d> newCOM(newCenterOfMass.data());

        newI =  R*oldIWrtCom*R.transpose() - newMass*squareCrossProductMatrix(newCOM);

        return SpatialInertia(newMass,newCenterOfMass,newRotInertia);
    }

    template<>
    ArticulatedBodyInertia transform(const Transform& op1, const ArticulatedBodyInertia& op2)
    {
        /*
         * The transform of a Articulated Body  Inertia is defined as follows:
         * A^I = A^X_B^* * B^I * B^X_A
         */
        Eigen::Map<const Matrix3dRowMajor> R(op1.getRotation().data());
        Eigen::Map<const Eigen::Vector3d> p(op1.getPosition().data());

        /*
         *
         * TODO: document this simplifcation
         */
        Eigen::Map<const Matrix3dRowMajor> oldLinearLinear(op2.getLinearLinearSubmatrix().data());
        Eigen::Map<const Matrix3dRowMajor> oldLinearAngular(op2.getLinearAngularSubmatrix().data());
        Eigen::Map<const Matrix3dRowMajor> oldAngularAngular(op2.getAngularAngularSubmatrix().data());

        ArticulatedBodyInertia newABI;

        Eigen::Map<Matrix3dRowMajor> newLinearLinear(newABI.getLinearLinearSubmatrix().data());
        Eigen::Map<Matrix3dRowMajor> newLinearAngular(newABI.getLinearAngularSubmatrix().data());
        Eigen::Map<Matrix3dRowMajor> newAngularAngular(newABI.getAngularAngularSubmatrix().data());

        newLinearLinear = R*oldLinearLinear*R.transpose();

        Matrix3dRowMajor skewP = mySkew(p);
        Matrix3dRowMajor rotatedLinearAngular = R*oldLinearAngular*R.transpose();

        newLinearAngular = rotatedLinearAngular - newLinearLinear*skewP;

        Matrix3dRowMajor skewPtimesrotatedLinearAngular = skewP*rotatedLinearAngular;

        newAngularAngular =   R*oldAngularAngular*R.transpose()
                            + skewPtimesrotatedLinearAngular
                            + skewPtimesrotatedLinearAngular.transpose()
                            - skewP*newLinearLinear*skewP;

        return newABI;
    }

    template<typename spatialVelType>
    spatialVelType transformTwistEfficient(const Transform& op1, const spatialVelType& op2)
    {
        spatialVelType ret;

        Eigen::Map<const Eigen::Vector3d> p(op1.getPosition().data());
        Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> > R(op1.getRotation().data());

        toEigen(ret.getAngularVec3()) = R*toEigen(op2.getAngularVec3());
        toEigen(ret.getLinearVec3())  = R*toEigen(op2.getLinearVec3()) + p.cross(toEigen(ret.getAngularVec3()));

        return ret;
    }

    template<typename spatialForceType>
    spatialForceType transformWrenchEfficient(const Transform& op1, const spatialForceType& op2)
    {
        spatialForceType ret;

        Eigen::Map<const Eigen::Vector3d> p(op1.getPosition().data());
        Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> > R(op1.getRotation().data());

        toEigen(ret.getLinearVec3()) = R*toEigen(op2.getLinearVec3());
        toEigen(ret.getAngularVec3())  = R*toEigen(op2.getAngularVec3()) + p.cross(toEigen(ret.getLinearVec3()));

        return ret;
    }

    Matrix4x4 Transform::asHomogeneousTransform() const
    {
        Matrix4x4 ret;

        Eigen::Map< Eigen::Matrix<double,4,4,Eigen::RowMajor> > retEigen(ret.data());

        Eigen::Map<const Eigen::Vector3d> p(this->getPosition().data());
        Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> > R(this->getRotation().data());

        retEigen.block<3,3>(0,0) = R;
        retEigen.block<3,1>(0,3) = p;
        retEigen.block<1,3>(3,0).setZero();
        retEigen(3,3) = 1;


        return ret;
    }

    Matrix6x6 Transform::asAdjointTransform() const
    {
        Matrix6x6 ret;

        Eigen::Map< Eigen::Matrix<double,6,6,Eigen::RowMajor> > retEigen(ret.data());

        Eigen::Map<const Eigen::Vector3d> p(this->getPosition().data());
        Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> > R(this->getRotation().data());

        retEigen.block<3,3>(0,0) = R;
        retEigen.block<3,3>(0,3) = mySkew(p)*R;
        retEigen.block<3,3>(3,0).setZero();
        retEigen.block<3,3>(3,3) = R;

        return ret;
    }

    Matrix6x6 Transform::asAdjointTransformWrench() const
    {
        Matrix6x6 ret;

        Eigen::Map< Eigen::Matrix<double,6,6,Eigen::RowMajor> > retEigen(ret.data());

        Eigen::Map<const Eigen::Vector3d> p(this->getPosition().data());
        Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> > R(this->getRotation().data());

        retEigen.block<3,3>(0,0) = R;
        retEigen.block<3,3>(0,3).setZero();
        retEigen.block<3,3>(3,0) = mySkew(p)*R;
        retEigen.block<3,3>(3,3) = R;

        return ret;
    }

    SpatialMotionVector Transform::log() const
    {
        SpatialMotionVector logRes;

        // the linear part is affected by the left Jacobian inverse of SO(3)
        auto omega = this->getRotation().log();
        auto JinvSO3 = Rotation::leftJacobianInverse(omega);
        Vector3 rho;
        toEigen(rho) = toEigen(JinvSO3)*toEigen(this->getPosition());
        memcpy(logRes.getLinearVec3().data(),rho.data(),3*sizeof(double));

        // the angular part instead mapped by SO(3) -> so(3) log
        logRes.setAngularVec3(this->getRotation().log());

        return logRes;
    }

}
