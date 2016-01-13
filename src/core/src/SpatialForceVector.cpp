/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/SpatialForceVector.h>
#include <iDynTree/Core/Utils.h>

#include <iDynTree/Core/EigenHelpers.h>

#include <sstream>

namespace iDynTree
{

SpatialForceVector::SpatialForceVector(const LinearForceVector3 & _linearVec3,
                                       const AngularForceVector3 & _angularVec3):
                                       SpatialVector<SpatialForceVector>(_linearVec3, _angularVec3)
{
}

SpatialForceVector::SpatialForceVector(const SpatialForceVector& other):
                                       SpatialVector<SpatialForceVector>(other)
{
}

SpatialForceVector::SpatialForceVector(const SpatialVector<SpatialForceVector>& other):
                                       SpatialVector<SpatialForceVector>(other)
{
}

SpatialForceVector::~SpatialForceVector()
{
}

SpatialForceVector SpatialForceVector::operator*(const double scalar) const
{
    SpatialForceVector scaledVec;

    toEigen(scaledVec.linearVec3)  = scalar*toEigen(this->linearVec3);
    toEigen(scaledVec.angularVec3) = scalar*toEigen(this->angularVec3);

    return scaledVec;
}



}
