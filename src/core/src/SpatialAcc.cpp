/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Core/SpatialAcc.h>
#include <iDynTree/Core/PrivateUtils.h>

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
    return efficient6dSum(*this,other);
}

SpatialAcc SpatialAcc::operator-() const
{
    return inverse(*this);
}

SpatialAcc SpatialAcc::operator-(const SpatialAcc& other) const
{
    return efficient6ddifference(*this,other);
}

}
