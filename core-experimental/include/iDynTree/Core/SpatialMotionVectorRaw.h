/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_SPATIAL_MOTION_RAW_H
#define IDYNTREE_SPATIAL_MOTION_RAW_H

#include "Vector6.h"

namespace iDynTree
{
    class PositionRaw;
    class RotationRaw;
    class SpatialForceVectorRaw;

    /**
     * Class providing the raw coordinates for any motion spatial vector.
     *
     * \ingroup iDynTreeCore
     *
     * A motion spatial vector can be used to to described twist, twist acceleration,
     * and their derivatives.
     *
     * This is just a basic vector, used to implement the adjoint transformations in
     * a general way. The relative adjoint transformation is contained in
     * TransformRaw::apply(SpatialMotionRaw),
     * for consistency with the iDynTree::PositionRaw class.
     *
     * \note in iDynTree, the spatial vector follows this serialization: the first three elements are
     *       the linear part and the second three elements are the angular part.
     */
    class SpatialMotionVectorRaw: public Vector6
    {
    public:
        SpatialMotionVectorRaw();
        SpatialMotionVectorRaw(const double* in_data, const unsigned int in_size);
        SpatialMotionVectorRaw(const SpatialMotionVectorRaw & other);
        virtual ~SpatialMotionVectorRaw();

        const SpatialMotionVectorRaw & changePoint(const PositionRaw & newPoint);
        const SpatialMotionVectorRaw & changeCoordFrame(const RotationRaw & newCoordFrame);
        static SpatialMotionVectorRaw compose(const SpatialMotionVectorRaw & op1, const SpatialMotionVectorRaw & op2);
        static SpatialMotionVectorRaw inverse(const SpatialMotionVectorRaw & op);

        double dot(const SpatialForceVectorRaw& other) const;
        SpatialMotionVectorRaw operator+(const SpatialMotionVectorRaw &other) const;
        SpatialMotionVectorRaw operator-(const SpatialMotionVectorRaw &other) const;
        SpatialMotionVectorRaw operator-() const;
    };
}

#endif /* IDYNTREE_SPATIAL_MOTION_RAW_H */