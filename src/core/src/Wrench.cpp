// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/Core/Wrench.h>
#include <iDynTree/Core/PrivateUtils.h>

namespace iDynTree
{

Wrench::Wrench(const Force & _linearVec3,
               const Torque & _angularVec3):
               SpatialForceVector(_linearVec3, _angularVec3)
{

}

Wrench::Wrench(const SpatialForceVector& other):
               SpatialForceVector(other)
{

}


Wrench::Wrench(const Wrench& other):
               SpatialForceVector(other)
{
}

Wrench Wrench::operator+(const Wrench& other) const
{
    return efficient6dSum(*this,other);
}

Wrench Wrench::operator-() const
{
    return inverse(*this);
}

Wrench Wrench::operator-(const Wrench& other) const
{
    return efficient6ddifference(*this,other);
}

}
