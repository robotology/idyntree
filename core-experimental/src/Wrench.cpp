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

Wrench::Wrench(const double* in_data, const unsigned int in_size):
               SpatialForceVectorRaw(in_data, in_size)
{

}

Wrench::Wrench(const SpatialForceVectorRaw& other):
               SpatialForceVectorRaw(other)
{

}


Wrench::Wrench(const Wrench& other):
               SpatialForceVectorRaw(other.data(),6)
{

}


Wrench::~Wrench()
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