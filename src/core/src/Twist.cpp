// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/Twist.h>
#include <iDynTree/Wrench.h>
#include <iDynTree/SpatialAcc.h>
#include <iDynTree/SpatialMomentum.h>
#include <iDynTree/PrivateUtils.h>

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

Twist& Twist::operator=(const Twist &other)
{
    this->getLinearVec3() = other.getLinearVec3();
    this->getAngularVec3() = other.getAngularVec3();
    return *this;
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
