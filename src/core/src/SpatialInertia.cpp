// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/Position.h>
#include <iDynTree/PrivateUtils.h>
#include <iDynTree/SpatialInertia.h>
#include <iDynTree/SpatialMomentum.h>
#include <iDynTree/Twist.h>
#include <iDynTree/SpatialAcc.h>
#include <iDynTree/Wrench.h>

#include <Eigen/Dense>
#include <iDynTree/EigenHelpers.h>

#include <cassert>
#include <iostream>
#include <sstream>

namespace iDynTree
{

SpatialInertia::SpatialInertia(const double mass,
                                     const Position& com,
                                     const RotationalInertia& rotInertia): m_mass(mass),
                                                                              m_rotInertia(rotInertia)
{
    for(int i = 0; i < 3; i++ )
    {
        this->m_mcom[i] = this->m_mass*com(i);
    }
}

SpatialInertia::SpatialInertia(const SpatialInertia& other): m_mass(other.m_mass),
                                                                      m_rotInertia(other.m_rotInertia)
{
    for(int i = 0; i < 3; i++ )
    {
        m_mcom[i] = other.m_mcom[i];
    }
}

void SpatialInertia::fromRotationalInertiaWrtCenterOfMass(const double mass,
                                                        const Position& com,
                                                        const RotationalInertia& rotInertiaWrtCom)
{
    this->m_mass = mass;

    for(int i = 0; i < 3; i++ )
    {
        this->m_mcom[i] = this->m_mass*com(i);
    }

    // Here we need to compute the rotational inertia at the com
    // given the one expressed at the frame origin
    // we apply formula 2.63 in Featherstone 2008
    Eigen::Map<Eigen::Matrix3d> linkInertia(this->m_rotInertia.data());
    Eigen::Map<const Eigen::Matrix3d> comInertia(rotInertiaWrtCom.data());
    Eigen::Map<const Eigen::Vector3d> mcom(this->m_mcom);

    if( fabs(this->m_mass) > 0)
    {
        linkInertia = comInertia - squareCrossProductMatrix(mcom)/this->m_mass;
    }
    else
    {
        linkInertia = comInertia;
    }
}


double SpatialInertia::getMass() const
{
    return this->m_mass;
}

Position SpatialInertia::getCenterOfMass() const
{
    Position ret;

    if( fabs(this->m_mass) > 0 )
    {
        ret(0) = this->m_mcom[0]/this->m_mass;
        ret(1) = this->m_mcom[1]/this->m_mass;
        ret(2) = this->m_mcom[2]/this->m_mass;
    }
    else
    {
        ret.zero();
    }

    return ret;
}

const RotationalInertia& SpatialInertia::getRotationalInertiaWrtFrameOrigin() const
{
    return this->m_rotInertia;
}

RotationalInertia SpatialInertia::getRotationalInertiaWrtCenterOfMass() const
{
    RotationalInertia retComInertia;
    // Here we need to compute the rotational inertia at the com
    // given the one expressed at the frame origin
    // we apply formula 2.63 in Featherstone 2008
    Eigen::Map<const Eigen::Matrix3d> linkInertia(this->m_rotInertia.data());
    Eigen::Map<Eigen::Matrix3d> comInertia(retComInertia.data());
    Eigen::Map<const Eigen::Vector3d> mcom(this->m_mcom);

    if( fabs(this->m_mass) > 0 )
    {
        comInertia = linkInertia + squareCrossProductMatrix(mcom)/this->m_mass;
    }
    else
    {
        comInertia = linkInertia;
    }

    return retComInertia;
}

SpatialInertia SpatialInertia::combine(const SpatialInertia& op1,
                                                                const SpatialInertia& op2)
{
    SpatialInertia ret;
    // If the two inertia are expressed with the same orientation
    // and with respect to the same point (and this will be checked by
    // the semantic check) we just need to sum
    // the mass, the first moment of mass and the rotational inertia
    ret.m_mass = op1.m_mass + op2.m_mass;

    Eigen::Map<Eigen::Vector3d> retMcom(ret.m_mcom);
    Eigen::Map<const Eigen::Vector3d> op1Mcom(op1.m_mcom);
    Eigen::Map<const Eigen::Vector3d> op2Mcom(op2.m_mcom);

    retMcom = op1Mcom + op2Mcom;

    Eigen::Map<Eigen::Matrix3d> retRotInertia(ret.m_rotInertia.data());
    Eigen::Map<const Eigen::Matrix3d> op1RotInertia(op1.m_rotInertia.data());
    Eigen::Map<const Eigen::Matrix3d> op2RotInertia(op2.m_rotInertia.data());

    retRotInertia = op1RotInertia + op2RotInertia;

    return ret;
}


SpatialForceVector SpatialInertia::multiply(const SpatialMotionVector& op) const
{
    SpatialForceVector ret;

    // we call this linearForce and angularForce
    // but please remember that they can also be
    // linear and angular momentum
    Eigen::Map<Eigen::Vector3d> linearForce(ret.getLinearVec3().data());
    Eigen::Map<Eigen::Vector3d> angularForce(ret.getAngularVec3().data());
    Eigen::Map<const Eigen::Vector3d> linearMotion(op.getLinearVec3().data());
    Eigen::Map<const Eigen::Vector3d> angularMotion(op.getAngularVec3().data());

    Eigen::Map<const Eigen::Vector3d> mcom(this->m_mcom);
    Eigen::Map<const Eigen::Matrix3d> inertia3d(this->m_rotInertia.data());

    // Implementing the 2.63 formula in Featherstone 2008
    linearForce  = this->m_mass*linearMotion - mcom.cross(angularMotion);
    angularForce = mcom.cross(linearMotion) + inertia3d*(angularMotion);

    return ret;
}


void SpatialInertia::zero()
{
    m_mass = 0.0;
    for(int i = 0; i < 3; i++ )
    {
        this->m_mcom[i] = 0.0;
    }
    this->m_rotInertia.zero();
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

Twist SpatialInertia::applyInverse(const SpatialMomentum& mom) const
{
    Twist vel;

    Eigen::Matrix<double,6,1> momEigen = toEigen(mom.asVector());

    Matrix6x6 I = this->asMatrix();
    Eigen::Matrix<double,6,1> velEigen = toEigen(I).householderQr().solve(momEigen);

    toEigen(vel.getLinearVec3()) = velEigen.block<3,1>(0,0);
    toEigen(vel.getAngularVec3()) = velEigen.block<3,1>(3,0);

    return vel;
}

Matrix6x6 SpatialInertia::getInverse() const
{
    Matrix6x6 ret;
    Matrix6x6 In = this->asMatrix();

    toEigen(ret) = toEigen(In).inverse();

    return ret;
}


SpatialInertia SpatialInertia::operator+(const SpatialInertia& other) const
{
    return SpatialInertia::combine(*this,other);
}

SpatialForceVector SpatialInertia::operator*(const SpatialMotionVector& other) const
{
    return SpatialInertia::multiply(other);
}

Wrench SpatialInertia::operator*(const SpatialAcc& other) const
{
    return SpatialInertia::multiply(other);
}

SpatialMomentum SpatialInertia::operator*(const Twist& other) const
{
    return SpatialInertia::multiply(other);
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

    Eigen::Map< Eigen::Matrix<double,10,1> > inertialParams = toEigen(ret);

    // Mass
    inertialParams(0) = m_mass;

    // First moment of mass
    inertialParams(1) = m_mcom[0];
    inertialParams(2) = m_mcom[1];
    inertialParams(3) = m_mcom[2];

    // Inertia matrix
    inertialParams(4) = m_rotInertia(0,0);
    inertialParams(5) = m_rotInertia(0,1);
    inertialParams(6) = m_rotInertia(0,2);
    inertialParams(7) = m_rotInertia(1,1);
    inertialParams(8) = m_rotInertia(1,2);
    inertialParams(9) = m_rotInertia(2,2);

    return ret;
}

void SpatialInertia::fromVector(const Vector10& inertialParams)
{
    // mass
    this->m_mass = inertialParams(0);

    // First moment of mass
    this->m_mcom[0] = inertialParams(1);
    this->m_mcom[1] = inertialParams(2);
    this->m_mcom[2] = inertialParams(3);

    // Inertia matrix
    this->m_rotInertia(0,0) = inertialParams(4);
    this->m_rotInertia(0,1) = this->m_rotInertia(1,0) = inertialParams(5);
    this->m_rotInertia(0,2) = this->m_rotInertia(2,0) = inertialParams(6);
    this->m_rotInertia(1,1) = inertialParams(7);
    this->m_rotInertia(1,2) = this->m_rotInertia(2,1) = inertialParams(8);
    this->m_rotInertia(2,2) = inertialParams(9);
}

bool SpatialInertia::isPhysicallyConsistent() const
{
    using namespace Eigen;

    bool isConsistent = true;

    // check that the mass is positive
    if( this->m_mass <= 0 )
    {
        isConsistent = false;
        return isConsistent;
    }

    // We get the inertia at the COM
    RotationalInertia inertiaAtCOM = this->getRotationalInertiaWrtCenterOfMass();

    // We get the inertia at the principal axis using eigen
    SelfAdjointEigenSolver<Matrix<double,3,3,RowMajor> > eigenValuesSolver;

    // We check both the positive definitiveness of the matrix and the triangle
    // inequality by directly checking the central second moment of mass of the rigid body
    // In a nutshell, we have that:
    // Ixx = Cyy + Czz
    // Iyy = Cxx + Czz
    // Izz = Cxx + Cyy
    // so
    // Cxx = (Iyy + Izz - Ixx)/2
    // Cyy = (Ixx + Izz - Iyy)/2
    // Czz = (Ixx + Iyy - Izz)/2
    // Then all the condition boils down to:
    // Cxx >= 0 , Cyy >= 0 , Czz >= 0
    eigenValuesSolver.compute( toEigen(inertiaAtCOM) );

    double Ixx = eigenValuesSolver.eigenvalues()(0);
    double Iyy = eigenValuesSolver.eigenvalues()(1);
    double Izz = eigenValuesSolver.eigenvalues()(2);

    double Cxx = (Iyy + Izz - Ixx)/2;
    double Cyy = (Ixx + Izz - Iyy)/2;
    double Czz = (Ixx + Iyy - Izz)/2;

    // We are accepting the configuration where C** is zero
    // so that we don't mark a point mass as physically inconsistent
    if( Cxx < 0 ||
        Cyy < 0 ||
        Czz < 0 )
    {
        isConsistent = false;
    }

    return isConsistent;
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

    Map< Matrix<double,6,10, Eigen::RowMajor> > res = toEigen(ret);

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
