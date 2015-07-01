/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "SpatialInertiaRaw.h"
#include "PositionRaw.h"
#include "Utils.h"

#include <Eigen/Dense>

#include <cassert>
#include <iostream>
#include <sstream>


namespace iDynTree
{

/**
 * Maps a 3d vector to the square of the cross product matrix:
 * v --> (v\times)^2
 * or, if you prefer another notation:
 * v --> S^2(v)
 */
Eigen::Matrix3d squareCrossProductMatrix(const Eigen::Vector3d & v)
{
    Eigen::Matrix3d ret;

    double vSqr[3];
    vSqr[0] = v[0]*v[0];
    vSqr[1] = v[1]*v[1];
    vSqr[2] = v[2]*v[2];

    ret <<  -(vSqr[1]+vSqr[2]),         v[0]*v[1],          v[0]*v[2],
                    v[0]*v[1], -(vSqr[0]+vSqr[2]),          v[1]*v[2],
                    v[0]*v[2],          v[1]*v[2], -(vSqr[0]+vSqr[1]);

    return ret;
}

SpatialInertiaRaw::SpatialInertiaRaw()
{
    this->zero();
}

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

SpatialInertiaRaw::~SpatialInertiaRaw()
{

}

double SpatialInertiaRaw::getMass() const
{
    return this->m_mass;
}

PositionRaw SpatialInertiaRaw::getCenterOfMass() const
{
    PositionRaw ret;
    ret(0) = this->m_mcom[0]/this->m_mass;
    ret(1) = this->m_mcom[1]/this->m_mass;
    ret(2) = this->m_mcom[2]/this->m_mass;

    return ret;
}

RotationalInertiaRaw SpatialInertiaRaw::getRotationalInertiaWrtFrameOrigin() const
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

    comInertia = linkInertia + squareCrossProductMatrix(mcom)/this->m_mass;

    return retComInertia;
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