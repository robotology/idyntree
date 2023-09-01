// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

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
