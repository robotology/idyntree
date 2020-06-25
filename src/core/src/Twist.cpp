/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
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
    return efficient6dSum(*this,other);
}

Twist Twist::operator-() const
{
    return inverse(*this);
}

Twist Twist::operator-(const Twist& other) const
{
    return efficient6ddifference(*this,other);
}

Wrench Twist::operator*(const SpatialMomentum& other) const
{
    return efficientTwistCrossMomentum<Twist,SpatialMomentum,Wrench>(*this,other);
}

SpatialAcc Twist::operator*(const Twist& other) const
{
    return efficientTwistCrossTwist<Twist,Twist,SpatialAcc>(*this,other);
}


}
