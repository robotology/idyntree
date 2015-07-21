/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/SpatialInertia.h>
#include <iDynTree/Core/SpatialMomentum.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/SpatialAcc.h>
#include <iDynTree/Core/Wrench.h>

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

SpatialInertia SpatialInertia::combine(const SpatialInertia& op1, const SpatialInertia& op2)
{
    return SpatialInertiaRaw::combine(op1,op2);
}

SpatialInertia SpatialInertia::operator+(const SpatialInertia& other) const
{
    return SpatialInertia::combine(*this,other);
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