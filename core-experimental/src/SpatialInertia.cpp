/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "SpatialInertia.h"
#include "SpatialMomentum.h"
#include "Twist.h"
#include "SpatialAcc.h"
#include "Wrench.h"

#include <cassert>
#include <iostream>
#include <sstream>

namespace iDynTree
{

SpatialInertia::SpatialInertia()
{
    SpatialInertiaRaw::zero();
}

SpatialInertia::SpatialInertia(const double mass,
                               const PositionRaw& com,
                               const RotationalInertiaRaw& rotInertia): SpatialInertiaRaw(mass, com, rotInertia)
{

}

SpatialInertia::SpatialInertia(const SpatialInertiaRaw& other): SpatialInertiaRaw(other)
{

}


SpatialInertia::SpatialInertia(const SpatialInertia& other)
{

}

SpatialInertia::~SpatialInertia()
{

}

Wrench SpatialInertia::operator*(const SpatialAcc& other) const
{
    return SpatialInertiaRaw::multiply(other);
}

SpatialMomentum SpatialInertia::operator*(const Twist& other) const
{
    return SpatialInertiaRaw::multiply(other);
}


}