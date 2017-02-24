/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_SPATIALMOMENTUM_H
#define IDYNTREE_SPATIALMOMENTUM_H


#include <iDynTree/Core/SpatialForceVector.h>

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
    class SpatialMomentum: public SpatialForceVector
    {
    public:
        /**
         * Default constructor.
         * The data is not reset to the zero for perfomance reason.
         * Please initialize the data in the class before any use.
         */
        inline SpatialMomentum() {}
        SpatialMomentum(const LinMomentum & _linearVec3, const AngMomentum & _angularVec3);
        SpatialMomentum(const SpatialForceVector & other);
        SpatialMomentum(const SpatialMomentum & other);

        // overloaded operators
        SpatialMomentum operator+(const SpatialMomentum &other) const;
        SpatialMomentum operator-(const SpatialMomentum &other) const;
        SpatialMomentum operator-() const;
    };
}

#endif
