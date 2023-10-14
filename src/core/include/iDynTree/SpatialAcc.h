// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_SPATIAL_ACC_H
#define IDYNTREE_SPATIAL_ACC_H

#include <iDynTree/SpatialMotionVector.h>

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
        SpatialAcc& operator=(const SpatialAcc &other);
        SpatialAcc operator+(const SpatialAcc &other) const;
        SpatialAcc operator-(const SpatialAcc &other) const;
        SpatialAcc operator-() const;

    };
}

#endif
