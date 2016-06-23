/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/Wrench.h>
#include <iDynTree/Core/PrivateSemanticsMacros.h>
#include <iDynTree/Core/PrivateUtils.h>

namespace iDynTree
{

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
#ifdef IDYNTREE_DONT_USE_SEMANTICS
    return efficient6dSum(*this,other);
#else
    return compose(*this,(other));
#endif
}

Wrench Wrench::operator-() const
{
    return inverse(*this);
}

Wrench Wrench::operator-(const Wrench& other) const
{
#ifdef IDYNTREE_DONT_USE_SEMANTICS
    return efficient6ddifference(*this,other);
#else
    return compose(*this,inverse(other));
#endif
}

}
