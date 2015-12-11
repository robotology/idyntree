/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/SpatialInertia.h>
#include <iDynTree/Core/SpatialMomentum.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/SpatialAcc.h>
#include <iDynTree/Core/Wrench.h>

#include <Eigen/Dense>

#include <cassert>
#include <iostream>
#include <sstream>

namespace iDynTree
{

SpatialInertia::SpatialInertia()
{
}

SpatialInertia::SpatialInertia(const double mass,
                               const PositionRaw& com,
                               const RotationalInertiaRaw& rotInertia): SpatialInertiaRaw(mass, com, rotInertia)
{

}

SpatialInertia::SpatialInertia(const SpatialInertiaRaw& other): SpatialInertiaRaw(other)
{

}


SpatialInertia::SpatialInertia(const SpatialInertia& other): SpatialInertiaRaw(other)
{

}

SpatialInertia::~SpatialInertia()
{

}

SpatialInertia SpatialInertia::combine(const SpatialInertia& op1, const SpatialInertia& op2)
{
    return SpatialInertiaRaw::combine(op1,op2);
}

// \todo TODO have a unique mySkew
template<class Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> mySkewIn(const Eigen::MatrixBase<Derived> & vec)
{
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
    return (Eigen::Matrix<typename Derived::Scalar, 3, 3>() << 0.0, -vec[2], vec[1], vec[2], 0.0, -vec[0], -vec[1], vec[0], 0.0).finished();
}

Matrix6x6 SpatialInertia::asMatrix() const
{
    Matrix6x6 ret;

    // Implementing the 2.63 formula in Featherstone 2008
    // compare with ::multiply method for consistency

    Eigen::Map< Eigen::Matrix<double,6,6,Eigen::RowMajor> > retEigen(ret.data());

    Eigen::Map<const Eigen::Vector3d> mcom(this->m_mcom);
    Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> > I(this->m_rotInertia.data());

    retEigen.block<3,3>(0,0) =  this->getMass()*Eigen::Matrix<double,3,3,Eigen::RowMajor>::Identity();
    retEigen.block<3,3>(0,3) = -mySkewIn(mcom);
    retEigen.block<3,3>(3,0) = mySkewIn(mcom);
    retEigen.block<3,3>(3,3) = I;

    return ret;
}

SpatialInertia SpatialInertia::operator+(const SpatialInertia& other) const
{
    return SpatialInertia::combine(*this,other);
}

SpatialForceVector SpatialInertia::operator*(const SpatialMotionVector& other) const
{
    return SpatialInertiaRaw::multiply(other);
}


Wrench SpatialInertia::operator*(const SpatialAcc& other) const
{
    return SpatialInertiaRaw::multiply(other);
}

SpatialMomentum SpatialInertia::operator*(const Twist& other) const
{
    return SpatialInertiaRaw::multiply(other);
}

SpatialInertia SpatialInertia::Zero()
{
    SpatialInertia ret;
    ret.zero();

    return ret;
}



}