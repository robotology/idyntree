/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/TransformDerivative.h>
#include <iDynTree/Core/Transform.h>

#include <Eigen/Dense>

#include <iDynTree/Core/EigenHelpers.h>

typedef Eigen::Matrix<double,3,3,Eigen::RowMajor> Matrix3dRowMajor;

namespace iDynTree
{

TransformDerivative::TransformDerivative(const Matrix3x3& _rotDeriv,
                                         const Vector3& posDeriv):
                                         rotDerivative(_rotDeriv),
                                         posDerivative(posDeriv)
{

}

TransformDerivative::TransformDerivative(const TransformDerivative& other):
                                              rotDerivative(other.rotDerivative),
                                              posDerivative(other.posDerivative)
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



}
