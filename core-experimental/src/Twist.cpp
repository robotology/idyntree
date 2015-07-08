/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "Twist.h"
#include "Wrench.h"
#include "SpatialAcc.h"
#include "SpatialMomentum.h"

namespace iDynTree
{

Twist::Twist()
{

}

Twist::Twist(const double* in_data,
             const unsigned int in_size):
             SpatialMotionVectorRaw(in_data, in_size)
{

}

Twist::Twist(const SpatialMotionVectorRaw& other):
             SpatialMotionVectorRaw(other)
{

}


Twist::Twist(const Twist& other):
            SpatialMotionVectorRaw(other.data(),6)
{

}

Twist::~Twist()
{

}

Twist Twist::operator+(const Twist& other) const
{
    return compose(*this,other);
}

Twist Twist::operator-() const
{
    return inverse(*this);
}

Twist Twist::operator-(const Twist& other) const
{
    return compose(*this,inverse(other));
}

Wrench Twist::operator*(const SpatialMomentum& other) const
{
    return SpatialMotionVectorRaw::cross(other);

}

SpatialAcc Twist::operator*(const Twist& other) const
{
    return SpatialMotionVectorRaw::cross(other);
}


}