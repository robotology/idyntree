/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */


#include <iDynTree/Core/SpatialInertiaRaw.h>
#include <iDynTree/Core/SpatialForceVector.h>
#include <iDynTree/Core/SpatialMotionVector.h>
#include <iDynTree/Core/PositionRaw.h>
#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/PrivateUtils.h>

#include <Eigen/Dense>

#include <iostream>
#include <sstream>

#include <cassert>


namespace iDynTree
{
SpatialInertiaRaw::SpatialInertiaRaw(const double mass,
                                     const PositionRaw& com,
                                     const RotationalInertiaRaw& rotInertia): m_mass(mass),
                                                                              m_rotInertia(rotInertia)
{
    for(int i = 0; i < 3; i++ )
    {
        this->m_mcom[i] = this->m_mass*com(i);
    }
}

SpatialInertiaRaw::SpatialInertiaRaw(const SpatialInertiaRaw& other): m_mass(other.m_mass),
                                                                      m_rotInertia(other.m_rotInertia)
{
    for(int i = 0; i < 3; i++ )
    {
        m_mcom[i] = other.m_mcom[i];
    }
}

void SpatialInertiaRaw::fromRotationalInertiaWrtCenterOfMass(const double mass,
                                                        const PositionRaw& com,
                                                        const RotationalInertiaRaw& rotInertiaWrtCom)
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


double SpatialInertiaRaw::getMass() const
{
    return this->m_mass;
}

PositionRaw SpatialInertiaRaw::getCenterOfMass() const
{
    PositionRaw ret;

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

const RotationalInertiaRaw& SpatialInertiaRaw::getRotationalInertiaWrtFrameOrigin() const
{
    return this->m_rotInertia;
}

RotationalInertiaRaw SpatialInertiaRaw::getRotationalInertiaWrtCenterOfMass() const
{
    RotationalInertiaRaw retComInertia;
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

SpatialInertiaRaw SpatialInertiaRaw::combine(const SpatialInertiaRaw& op1,
                                                                const SpatialInertiaRaw& op2)
{
    SpatialInertiaRaw ret;
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


SpatialForceVector SpatialInertiaRaw::multiply(const SpatialMotionVector& op) const
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


void SpatialInertiaRaw::zero()
{
    m_mass = 0.0;
    for(int i = 0; i < 3; i++ )
    {
        this->m_mcom[i] = 0.0;
    }
    this->m_rotInertia.zero();
}


}
