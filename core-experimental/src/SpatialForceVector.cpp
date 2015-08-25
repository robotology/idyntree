/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/SpatialForceVector.h>
#include <iDynTree/Core/Utils.h>
#include <sstream>

namespace iDynTree
{

SpatialForceVector::SpatialForceVector()
{
    // Base class SpatialVector<...> will be implicitly called
}


SpatialForceVector::SpatialForceVector(const LinearForceVector3 & _linearVec3,
                                       const AngularForceVector3 & _angularVec3):
                                       SpatialVector<SpatialForceVector, LinearForceVector3, AngularForceVector3>(_linearVec3, _angularVec3)
{
}

SpatialForceVector::SpatialForceVector(const SpatialForceVector& other):
                                       SpatialVector<SpatialForceVector, LinearForceVector3, AngularForceVector3>(other)
{
}


SpatialForceVector::~SpatialForceVector()
{
}

}
