/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

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
