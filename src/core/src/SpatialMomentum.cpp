// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/SpatialMomentum.h>
#include <iDynTree/PrivateUtils.h>

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

SpatialMomentum& SpatialMomentum::operator=(const SpatialMomentum &other)
{
    this->getLinearVec3() = other.getLinearVec3();
    this->getAngularVec3() = other.getAngularVec3();
    return *this;
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
