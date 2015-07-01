/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "SpatialInertiaRaw.h"
#include "PositionRaw.h"
#include "Utils.h"
#include <cassert>
#include <iostream>
#include <sstream>


namespace iDynTree
{

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