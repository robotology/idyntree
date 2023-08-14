// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

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
