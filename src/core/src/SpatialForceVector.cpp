/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
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
