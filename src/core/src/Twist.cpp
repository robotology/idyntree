/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/Wrench.h>
#include <iDynTree/Core/SpatialAcc.h>
#include <iDynTree/Core/SpatialMomentum.h>
#include <iDynTree/Core/PrivateUtils.h>

namespace iDynTree
{

Twist::Twist()
{

}

Twist::Twist(const LinVelocity & _linearVec3,
             const AngVelocity & _angularVec3):
             SpatialMotionVector(_linearVec3, _angularVec3)
{

}

Twist::Twist(const SpatialMotionVector& other):
             SpatialMotionVector(other)
{

}


Twist::Twist(const Twist& other):
             SpatialMotionVector(other)
{

}

Twist Twist::operator+(const Twist& other) const
{
#ifdef IDYNTREE_DONT_USE_SEMANTICS
    return efficient6dSum(*this,other);
#else
    return compose(*this,(other));
#endif
}

Twist Twist::operator-() const
{
    return inverse(*this);
}

Twist Twist::operator-(const Twist& other) const
{
#ifdef IDYNTREE_DONT_USE_SEMANTICS
    return efficient6ddifference(*this,other);
#else
    return compose(*this,inverse(other));
#endif
}

Wrench Twist::operator*(const SpatialMomentum& other) const
{
#ifdef IDYNTREE_DONT_USE_SEMANTICS
    return efficientTwistCrossMomentum<Twist,SpatialMomentum,Wrench>(*this,other);
#else
    return SpatialMotionVector::cross(other);
#endif
}

SpatialAcc Twist::operator*(const Twist& other) const
{
#ifdef IDYNTREE_DONT_USE_SEMANTICS
    return efficientTwistCrossTwist<Twist,Twist,SpatialAcc>(*this,other);
#else
    return SpatialMotionVector::cross(other);
#endif
}


}