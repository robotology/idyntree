/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
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
