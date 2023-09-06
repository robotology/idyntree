// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/ArticulatedBodyInertia.h>
#include <iDynTree/TransformDerivative.h>
#include <iDynTree/Transform.h>

#include <Eigen/Dense>

#include <iDynTree/EigenHelpers.h>

typedef Eigen::Matrix<double,3,3,Eigen::RowMajor> Matrix3dRowMajor;

namespace iDynTree
{

TransformDerivative::TransformDerivative(const Matrix3x3& _rotDeriv,
                                         const Vector3& posDeriv):
                                            posDerivative(posDeriv),
                                            rotDerivative(_rotDeriv)

{

}

TransformDerivative::TransformDerivative(const TransformDerivative& other):
                                              posDerivative(other.posDerivative),
                                              rotDerivative(other.rotDerivative)
{
}

const Matrix3x3& TransformDerivative::getRotationDerivative() const
{
    return this->rotDerivative;
}

const Vector3& TransformDerivative::getPositionDerivative() const
{
    return this->posDerivative;
}

void TransformDerivative::setRotationDerivative(const Matrix3x3& rotationDerivative)
{
    this->rotDerivative = rotationDerivative;
}

void TransformDerivative::setPositionDerivative(const Vector3& positionDerivative)
{
    this->posDerivative = positionDerivative;
}

TransformDerivative TransformDerivative::Zero()
{
    TransformDerivative ret;
    ret.rotDerivative.zero();
    ret.posDerivative.zero();
    return ret;
}

Matrix4x4 TransformDerivative::asHomogeneousTransformDerivative() const
{
    Matrix4x4 ret;

    Eigen::Map< Eigen::Matrix<double,4,4,Eigen::RowMajor> > retEigen(ret.data());

    Eigen::Map<const Eigen::Vector3d> dp(this->posDerivative.data());
    Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> > dR(this->rotDerivative.data());

    retEigen.block<3,3>(0,0) = dR;
    retEigen.block<3,1>(0,3) = dp;
    retEigen.block<1,4>(3,0).setZero();

    return ret;
}

// \todo TODO have a unique mySkew
template<class Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> mySkeww(const Eigen::MatrixBase<Derived> & vec)
{
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    return (Eigen::Matrix<typename Derived::Scalar, 3, 3>() << 0.0, -vec[2], vec[1], vec[2], 0.0, -vec[0], -vec[1], vec[0], 0.0).finished();
}


Matrix6x6 TransformDerivative::asAdjointTransformDerivative(const Transform& transform) const
{
    Matrix6x6 ret;

    Eigen::Map< Eigen::Matrix<double,6,6,Eigen::RowMajor> > retEigen(ret.data());

    Eigen::Map<const Eigen::Vector3d> dp(this->posDerivative.data());
    Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> > dR(this->rotDerivative.data());

    Eigen::Map<const Eigen::Vector3d> p(transform.getPosition().data());
    Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> > R(transform.getRotation().data());

    retEigen.block<3,3>(0,0) = dR;
    retEigen.block<3,3>(0,3) = mySkeww(dp)*R+mySkeww(p)*dR;
    retEigen.block<3,3>(3,0).setZero();
    retEigen.block<3,3>(3,3) = dR;

    return ret;
}

Matrix6x6 TransformDerivative::asAdjointTransformWrenchDerivative(const Transform& transform) const
{
    Matrix6x6 ret;

    Eigen::Map< Eigen::Matrix<double,6,6,Eigen::RowMajor> > retEigen(ret.data());

    Eigen::Map<const Eigen::Vector3d> dp(this->posDerivative.data());
    Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> > dR(this->rotDerivative.data());

    Eigen::Map<const Eigen::Vector3d> p(transform.getPosition().data());
    Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> > R(transform.getRotation().data());

    retEigen.block<3,3>(0,0) = dR;
    retEigen.block<3,3>(0,3).setZero();
    retEigen.block<3,3>(3,0) = mySkeww(dp)*R + mySkeww(p)*dR;
    retEigen.block<3,3>(3,3) = dR;

    return ret;
}

TransformDerivative TransformDerivative::operator*(const Transform& otherTransform) const
{
    TransformDerivative ret;

    Eigen::Map<const Eigen::Vector3d> dp(this->posDerivative.data());
    Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> > dR(this->rotDerivative.data());

    Eigen::Map<const Eigen::Vector3d> other_p(otherTransform.getPosition().data());
    Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> > other_R(otherTransform.getRotation().data());

    Eigen::Map<Eigen::Vector3d> ret_dp(ret.posDerivative.data());
    Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor> >ret_dR(ret.rotDerivative.data());

    ret_dR = dR*other_R;
    ret_dp = dR*other_p+dp;

    return ret;
}

TransformDerivative TransformDerivative::derivativeOfInverse(const Transform& a_H_b) const
{
    Transform b_H_a = a_H_b.inverse();

    TransformDerivative ret;

    // For the sake of compact notation, we will indicated
    // a_R_b the rotational part of a_H_b, and a_O_b the translation part of a_H_b)
    Eigen::Map<const Eigen::Vector3d> d_a_O_b(this->posDerivative.data());
    Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> > d_a_R_b(this->rotDerivative.data());

    Eigen::Map<const Eigen::Vector3d> b_O_a(b_H_a.getPosition().data());
    Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> > b_R_a(b_H_a.getRotation().data());

    Eigen::Map<Eigen::Vector3d> d_b_O_a(ret.posDerivative.data());
    Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor> > d_b_R_a(ret.rotDerivative.data());

    // ~^b \dot{R}_a = - b_R_a a_\dot{R}_b b_R_a
    d_b_R_a = -b_R_a*d_a_R_b*b_R_a;

    // ~^b \dot{O}_a = - b_R_a*(a_\dot{R}_b ~^b O_a + ~^a \dot{O}_b
    d_b_O_a = -b_R_a*(d_a_R_b*b_O_a+d_a_O_b);

    return ret;
}

ArticulatedBodyInertia TransformDerivative::transform(const Transform& transform,
                                                      ArticulatedBodyInertia& other)
{
    // Inefficient implementation for the time being, this should not be a bottleneck
    Matrix6x6 oldABI = other.asMatrix();
    Matrix6x6 a_X_b_wrench  = transform.asAdjointTransformWrench();
    Matrix6x6 a_dX_b_wrench = this->asAdjointTransformWrenchDerivative(transform);
    Eigen::Matrix<double,6,6,Eigen::RowMajor> b_X_a = toEigen(a_X_b_wrench).transpose();
    Eigen::Matrix<double,6,6,Eigen::RowMajor> b_dX_a = toEigen(a_dX_b_wrench).transpose();

    Matrix6x6 newABI;

    toEigen(newABI) =  toEigen(a_dX_b_wrench)*toEigen(oldABI)*(b_X_a)
                      +toEigen(a_X_b_wrench)*toEigen(oldABI)*(b_dX_a);

    return ArticulatedBodyInertia(newABI.data(),6,6);
}

SpatialForceVector TransformDerivative::transform(const Transform& transform,
                                                  SpatialForceVector& other)
{
    SpatialForceVector ret;

    Eigen::Map<const Eigen::Vector3d> p(transform.getPosition().data());
    Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> > R(transform.getRotation().data());
    Eigen::Map<const Eigen::Vector3d> dp(this->posDerivative.data());
    Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> > dR(this->rotDerivative.data());

    toEigen(ret.getLinearVec3()) = dR*toEigen(other.getLinearVec3());
    toEigen(ret.getAngularVec3())  =  dR*toEigen(other.getAngularVec3())
                                    + p.cross(toEigen(ret.getLinearVec3()))
                                    + dp.cross(R*toEigen(other.getLinearVec3()));

    return ret;
}

SpatialMotionVector TransformDerivative::transform(const Transform& transform,
                                                   SpatialMotionVector& other)
{
    SpatialMotionVector ret;

    Eigen::Map<const Eigen::Vector3d> p(transform.getPosition().data());
    Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> > R(transform.getRotation().data());
    Eigen::Map<const Eigen::Vector3d> dp(this->posDerivative.data());
    Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> > dR(this->rotDerivative.data());

    toEigen(ret.getAngularVec3()) = dR*toEigen(other.getAngularVec3());
    toEigen(ret.getLinearVec3())  = dR*toEigen(other.getLinearVec3())
                                   + p.cross(toEigen(ret.getAngularVec3()))
                                   + dp.cross(R*toEigen(other.getAngularVec3()));


    return ret;
}





}
