/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "SpatialAcc.h"

namespace iDynTree
{

SpatialAcc::SpatialAcc()
{

}

SpatialAcc::SpatialAcc(const LinAcceleration & _linearVec3,
                       const AngAcceleration & _angularVec3):
                       SpatialMotionVector(_linearVec3, _angularVec3)
{

}

SpatialAcc::SpatialAcc(const SpatialMotionVector& other):
                       SpatialMotionVector(other)
{

}


SpatialAcc::SpatialAcc(const SpatialAcc& other):
                       SpatialMotionVector(other)
{

}


SpatialAcc::~SpatialAcc()
{

}

SpatialAcc SpatialAcc::operator+(const SpatialAcc& other) const
{
    return compose(*this,other);
}

SpatialAcc SpatialAcc::operator-() const
{
    return inverse(*this);
}

SpatialAcc SpatialAcc::operator-(const SpatialAcc& other) const
{
    return compose(*this,inverse(other));
}

}