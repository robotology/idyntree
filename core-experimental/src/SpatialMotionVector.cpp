/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "SpatialMotionVector.h"
#include "SpatialForceVector.h"
#include "Utils.h"
#include <sstream>
#include <Eigen/Dense>

namespace iDynTree
{

SpatialMotionVector::SpatialMotionVector()
{
    // Base class SpatialVector<...> will be implicitly called
}


SpatialMotionVector::SpatialMotionVector(const LinearMotionVector3 & _linearVec3,
                                         const AngularMotionVector3 & _angularVec3):
                                         SpatialVector<SpatialMotionVector, LinearMotionVector3, AngularMotionVector3>(_linearVec3, _angularVec3)
{
}

SpatialMotionVector::SpatialMotionVector(const SpatialMotionVector& other):
                                         SpatialVector<SpatialMotionVector, LinearMotionVector3, AngularMotionVector3>(other)
{
}


SpatialMotionVector::~SpatialMotionVector()
{
}

SpatialMotionVector SpatialMotionVector::cross(const SpatialMotionVector& other) const
{
    SpatialMotionVector res;
#if 0
    res.linearVec3  = this->angularVec3.cross(other->linearVec3) + this->linearVec3.cross(other->angularVec3);
    res.angularVec3 =                                              this->angularVec3.cross(other->angularVec3);

#endif
    return res;
}

SpatialForceVector SpatialMotionVector::cross(const SpatialForceVector& other) const
{
    SpatialForceVector res;
#if 0
    res.linearVec3  = this->angularVec3.cross(other->linearVec3);
    res.angularVec3 = this->linearVec3.cross(other->linearVec3) + this->angularVec3.cross(other->angularVec3);

#endif
    return res;
}


}