/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_SPATIAL_FORCE_RAW_H
#define IDYNTREE_SPATIAL_FORCE_RAW_H

#include "Vector6.h"

namespace iDynTree
{
    class PositionRaw;
    class RotationRaw;
    class SpatialMotionVectorRaw;

    /**
     * Class providing the raw coordinates for any spatial force vector.
     *
     * \ingroup iDynTreeCore
     *
     * A force spatial vector can be used to to described spatial momentum, wrench,
     * or their derivatives.
     *
     * This is just a basic vector, used to implement the adjoint transformations in
     * a general way. The relative adjoint transformation is contained in
     * TransformRaw::apply(SpatialForceRaw),
     * for consistency with the iDynTree::PositionRaw class.
     *
     * \note in iDynTree, the spatial vector follows this serialization: the first three elements are
     *       the linear part and the second three elements are the angular part.
     */
    class SpatialForceVectorRaw: public Vector6
    {
    public:
        SpatialForceVectorRaw();
        SpatialForceVectorRaw(const double* in_data, const unsigned int in_size);
        SpatialForceVectorRaw(const SpatialForceVectorRaw & other);
        virtual ~SpatialForceVectorRaw();

        const SpatialForceVectorRaw & changePoint(const PositionRaw & newPoint);
        const SpatialForceVectorRaw & changeCoordFrame(const RotationRaw & newCoordFrame);
        static SpatialForceVectorRaw compose(const SpatialForceVectorRaw & op1, const SpatialForceVectorRaw & op2);
        static SpatialForceVectorRaw inverse(const SpatialForceVectorRaw & op);

        double dot(const SpatialMotionVectorRaw & other) const;

        /** constructor helpers */
        static SpatialForceVectorRaw Zero();
    };
}

#endif /* IDYNTREE_SPATIAL_FORCE_RAW_H */