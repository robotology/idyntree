/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "SpatialMomentum.h"

namespace iDynTree
{

SpatialMomentum::SpatialMomentum()
{

}

SpatialMomentum::SpatialMomentum(const LinMomentum & _linearVec3,
                                 const AngMomentum & _angularVec3):
                                 SpatialForceVector(in_data, in_size)
{

}

SpatialMomentum::SpatialMomentum(const SpatialForceVector& other):
                                 SpatialForceVector(other)
{

}


SpatialMomentum::SpatialMomentum(const SpatialMomentum& other):
                                 SpatialForceVector(other.data(),6)
{

}

SpatialMomentum::~SpatialMomentum()
{

}

SpatialMomentum SpatialMomentum::operator+(const SpatialMomentum& other) const
{
    return compose(*this,other);
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