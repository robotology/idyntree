/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/SpatialMotionVector.h>
#include <iDynTree/Core/SpatialForceVector.h>
#include <iDynTree/Core/Transform.h>
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

Transform SpatialMotionVector::exp() const
{
    Transform res;

    // Understand if there is a meaningful
    // semantics for this operation and if it exists use it

    // the linear part is not changed by the exp
    Position newPos;
    memcpy(newPos.data(),this->getLinearVec3().data(),3*sizeof(double));
    res.setPosition(newPos);

    // the angular part instead mapped by so(3) -> SO(3) exp map
    res.setRotation(this->getAngularVec3().exp());

    return res;
}



}