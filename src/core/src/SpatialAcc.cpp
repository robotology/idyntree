/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/SpatialAcc.h>

namespace iDynTree
{
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

SpatialAcc SpatialAcc::operator+(const SpatialAcc& other) const
{
#ifdef IDYNTREE_DONT_USE_SEMANTICSD
    return efficient6dSum(*this,other);
#else
    return compose(*this,(other));
#endif
}

SpatialAcc SpatialAcc::operator-() const
{
    return inverse(*this);
}

SpatialAcc SpatialAcc::operator-(const SpatialAcc& other) const
{
#ifdef IDYNTREE_DONT_USE_SEMANTICSF
    return efficient6ddifference(*this,other);
#else
    return compose(*this,inverse(other));
#endif
}

}
