/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_SPATIALMOMENTUM_H
#define IDYNTREE_SPATIALMOMENTUM_H

#include <iDynTree/Core/SpatialForceVectorRaw.h>

namespace iDynTree
{
    /**
     * Class representing a spatial momentum,
     * i.e. a 6D combination of linear and angular momentum.
     *
     * \ingroup iDynTreeCore
     *
     * Currently this class does not support semantics.
     */
    class SpatialMomentum: public SpatialForceVectorRaw
    {
    public:
        SpatialMomentum();
        SpatialMomentum(const double* in_data, const unsigned int in_size);
        SpatialMomentum(const SpatialForceVectorRaw & other);
        SpatialMomentum(const SpatialMomentum & other);
        virtual ~SpatialMomentum();

        // overloaded operators
        SpatialMomentum operator+(const SpatialMomentum &other) const;
        SpatialMomentum operator-(const SpatialMomentum &other) const;
        SpatialMomentum operator-() const;
    };
}

#endif