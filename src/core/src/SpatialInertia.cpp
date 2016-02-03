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
#include <iDynTree/Core/EigenHelpers.h>

#include <cassert>
#include <iostream>
#include <sstream>

namespace iDynTree
{

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

Wrench SpatialInertia::biasWrench(const Twist& V) const
{
    Wrench ret;

    Eigen::Map<Eigen::Vector3d> linearBiasForce(ret.getLinearVec3().data());
    Eigen::Map<Eigen::Vector3d> angularBiasForce(ret.getAngularVec3().data());

    Eigen::Map<const Eigen::Vector3d> linearVel(V.getLinearVec3().data());
    Eigen::Map<const Eigen::Vector3d> angularVel(V.getAngularVec3().data());

    Eigen::Map<const Eigen::Vector3d> mcom(this->m_mcom);
    Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> > I(this->m_rotInertia.data());

    linearBiasForce = this->m_mass*(angularVel.cross(linearVel)) - angularVel.cross(mcom.cross(angularVel));
    angularBiasForce = mcom.cross(angularVel.cross(linearVel)) + angularVel.cross(I*angularVel);

    return ret;
}

Matrix6x6 SpatialInertia::biasWrenchDerivative(const Twist& V) const
{
    Matrix6x6 ret;

    Eigen::Map<const Eigen::Vector3d> linearVel(V.getLinearVec3().data());
    Eigen::Map<const Eigen::Vector3d> angularVel(V.getAngularVec3().data());

    Eigen::Map< Eigen::Matrix<double,6,6,Eigen::RowMajor> > retEigen(ret.data());

    Eigen::Map<const Eigen::Vector3d> mcom(this->m_mcom);
    Eigen::Map<const Eigen::Matrix<double,3,3,Eigen::RowMajor> > I(this->m_rotInertia.data());

    Eigen::Matrix<double,3,3,Eigen::RowMajor> mcCrossOmegaCross = mySkewIn(mcom)*mySkewIn(angularVel);
    retEigen.block<3,3>(0,0) = mySkewIn(this->m_mass*angularVel);
    retEigen.block<3,3>(0,3) = -mySkewIn(this->m_mass*linearVel) + mySkewIn(mcom.cross(angularVel)) - mcCrossOmegaCross.transpose();
    retEigen.block<3,3>(3,0) = mcCrossOmegaCross;
    retEigen.block<3,3>(3,3) = -mySkewIn(mcom)*mySkewIn(linearVel) + mySkewIn(angularVel)*I - mySkewIn(I*angularVel);

    return ret;
}

SpatialInertia SpatialInertia::Zero()
{
    SpatialInertia ret;
    ret.zero();

    return ret;
}

Vector10 SpatialInertia::asVector() const
{
    Vector10 ret;

    Eigen::Map< Eigen::Matrix<double,10,1> > res = toEigen(ret);

    // Mass
    res(0) = m_mass;

    // First moment of mass
    res(1) = m_mcom[0];
    res(2) = m_mcom[1];
    res(3) = m_mcom[2];

    // Inertia matrix
    res(4) = m_rotInertia(0,0);
    res(5) = m_rotInertia(0,1);
    res(6) = m_rotInertia(0,2);
    res(7) = m_rotInertia(1,1);
    res(8) = m_rotInertia(1,2);
    res(9) = m_rotInertia(2,2);

    return ret;
}

Eigen::Matrix<double, 3, 6> rotationalMomentumRegressor(const Vector3 & w)
{
    Eigen::Matrix<double, 3, 6> ret;

    ret << w(0), w(1), w(2),    0,    0,    0,
              0, w(0),    0, w(1), w(2),    0,
              0,    0, w(0),    0, w(1), w(2);

    return ret;
}

Matrix6x10 SpatialInertia::momentumRegressor(const Twist& v)
{
    using namespace Eigen;

    Matrix6x10 ret;

    Map< Matrix<double,6,10, RowMajor> > res = toEigen(ret);

    res <<  toEigen(v.getLinearVec3()),  mySkewIn(toEigen(v.getAngularVec3())), Matrix<double, 3, 6>::Zero(),
                    Vector3d::Zero(), -mySkewIn(toEigen(v.getLinearVec3())), rotationalMomentumRegressor(v.getAngularVec3());

    return ret;
}

Matrix6x10 SpatialInertia::momentumDerivativeRegressor(const Twist& v,
                                                       const SpatialAcc& a)
{
    Matrix6x10 ret;

    Eigen::Map< Eigen::Matrix<double,6,10,Eigen::RowMajor> > res = toEigen(ret);

    res = toEigen(momentumRegressor(a)) +
          toEigen(v.asCrossProductMatrixWrench())*toEigen(momentumRegressor(v));

    return ret;
}

Matrix6x10 SpatialInertia::momentumDerivativeSlotineLiRegressor(const Twist& v,
                                                                const Twist& vRef,
                                                                const SpatialAcc& aRef)
{
    Matrix6x10 ret;

    Eigen::Map< Eigen::Matrix<double,6,10,Eigen::RowMajor> > res = toEigen(ret);

    // The momentum derivative according to the Slotine-Li regressor is given by:
    // \dot{h} = I*a + v.crossWrench(I*vRef) - I*(v.cross(vRef))

    res = toEigen(momentumRegressor(aRef)) +
          toEigen(v.asCrossProductMatrixWrench())*toEigen(momentumRegressor(vRef)) -
          toEigen(momentumRegressor(v.cross(vRef)));

    return ret;
}




}