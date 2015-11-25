/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/Wrench.h>

namespace iDynTree
{

Wrench::Wrench()
{

}

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
    return compose(*this,other);
}

Wrench Wrench::operator-() const
{
    return inverse(*this);
}

Wrench Wrench::operator-(const Wrench& other) const
{
    return compose(*this,inverse(other));
}

}