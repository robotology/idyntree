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

}
