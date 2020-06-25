/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Core/SpatialMomentum.h>
#include <iDynTree/Core/PrivateUtils.h>

namespace iDynTree
{
SpatialMomentum::SpatialMomentum(const LinMomentum & _linearVec3,
                                 const AngMomentum & _angularVec3):
                                 SpatialForceVector(_linearVec3, _angularVec3)
{

}

SpatialMomentum::SpatialMomentum(const SpatialForceVector& other):
                                 SpatialForceVector(other)
{

}


SpatialMomentum::SpatialMomentum(const SpatialMomentum& other):
                                 SpatialForceVector(other)
{

}

SpatialMomentum SpatialMomentum::operator+(const SpatialMomentum& other) const
{
    return efficient6dSum(*this,other);
}

SpatialMomentum SpatialMomentum::operator-() const
{
    return inverse(*this);
}

SpatialMomentum SpatialMomentum::operator-(const SpatialMomentum& other) const
{
    return compose(*this,inverse(other));
}

}
