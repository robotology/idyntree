/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_SPATIAL_ACC_H
#define IDYNTREE_SPATIAL_ACC_H

#include "SpatialMotionVector.h"

namespace iDynTree
{
    /**
     * Class representing a spatial acceleration, i.e. the representation
     * of the time derivative of the twist.
     *
     * \note The linear part of this spatial vector **is not** the acceleration
     *       of a point.
     *
     * Currently this class does not support semantics.
     *
     * \ingroup iDynTreeCore
     */
    class SpatialAcc: public SpatialMotionVector
    {
    public:
        SpatialAcc();
        SpatialAcc(const LinAcceleration & _linearVec3, const AngAcceleration & _angularVec3);
        SpatialAcc(const SpatialMotionVector& other);
        SpatialAcc(const SpatialAcc& other);
        virtual ~SpatialAcc();

        // overloaded operator
        SpatialAcc operator+(const SpatialAcc &other) const;
        SpatialAcc operator-(const SpatialAcc &other) const;
        SpatialAcc operator-() const;

    };
}

#endif