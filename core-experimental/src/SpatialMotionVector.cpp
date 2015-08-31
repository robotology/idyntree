/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/SpatialMotionVector.h>
#include <iDynTree/Core/SpatialForceVector.h>
#include <iDynTree/Core/Utils.h>

#include <Eigen/Dense>

#include <sstream>


namespace iDynTree
{

SpatialMotionVector::SpatialMotionVector()
{
    // Base class SpatialVector<...> will be implicitly called
}


SpatialMotionVector::SpatialMotionVector(const LinearMotionVector3 & _linearVec3,
                                         const AngularMotionVector3 & _angularVec3):
                                         SpatialVector<SpatialMotionVector>(_linearVec3, _angularVec3)
{
}

SpatialMotionVector::SpatialMotionVector(const SpatialMotionVector& other):
                                         SpatialVector<SpatialMotionVector>(other)
{
}

SpatialMotionVector::SpatialMotionVector(const SpatialVector<SpatialMotionVector>& other):
                                         SpatialVector<SpatialMotionVector>(other)
{
}
    
SpatialMotionVector::~SpatialMotionVector()
{
}

SpatialMotionVector SpatialMotionVector::cross(const SpatialMotionVector& other) const
{
    SpatialMotionVector res;

    res.getLinearVec3()  = this->angularVec3.cross(other.getLinearVec3()) + this->linearVec3.cross(other.getAngularVec3());
    res.getAngularVec3() =                                                  this->angularVec3.cross(other.getAngularVec3());

    return res;
}

SpatialForceVector SpatialMotionVector::cross(const SpatialForceVector& other) const
{
    SpatialForceVector res;

    res.getLinearVec3()  = this->angularVec3.cross(other.getLinearVec3());
    res.getAngularVec3() = this->linearVec3.cross(other.getLinearVec3()) + this->angularVec3.cross(other.getAngularVec3());

    return res;
}


}