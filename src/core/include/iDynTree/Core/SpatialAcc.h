/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_SPATIAL_ACC_H
#define IDYNTREE_SPATIAL_ACC_H

#include <iDynTree/Core/SpatialMotionVector.h>

namespace iDynTree
{
    /**
     * Class representing a spatial acceleration, i.e. the representation
     * of the time derivative of the twist.
     *
     * \note The linear part of this spatial vector **is not** the acceleration
     *       of a point.
     *
     *
     * \ingroup iDynTreeCore
     */
    class SpatialAcc: public SpatialMotionVector
    {
    public:
        /**
         * Default constructor.
         * The data is not reset to zero for perfomance reason.
         * Please initialize the data in the class before any use.
         */
        inline SpatialAcc() {}
        SpatialAcc(const LinAcceleration & _linearVec3, const AngAcceleration & _angularVec3);
        SpatialAcc(const SpatialMotionVector& other);
        SpatialAcc(const SpatialAcc& other);

        // overloaded operator
        SpatialAcc operator+(const SpatialAcc &other) const;
        SpatialAcc operator-(const SpatialAcc &other) const;
        SpatialAcc operator-() const;

    };
}

#endif
