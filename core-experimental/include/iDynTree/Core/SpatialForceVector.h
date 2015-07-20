/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_SPATIAL_FORCE_RAW_H
#define IDYNTREE_SPATIAL_FORCE_RAW_H

#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/LinearForceVector3.h>
#include <iDynTree/Core/AngularForceVector3.h>
//#include <iDynTree/Core/ISpatialVector.h>

namespace iDynTree
{
    class PositionRaw;
    class RotationRaw;
    class SpatialMotionVector;

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
    class SpatialForceVector: public Vector6
    {
    public:
        SpatialForceVector();
        SpatialForceVector(const double* in_data, const unsigned int in_size);
        SpatialForceVector(const SpatialForceVector & other);
        virtual ~SpatialForceVector();

        const SpatialForceVector & changePoint(const PositionRaw & newPoint);
        const SpatialForceVector & changeCoordFrame(const RotationRaw & newCoordFrame);
        static SpatialForceVector compose(const SpatialForceVector & op1, const SpatialForceVector & op2);
        static SpatialForceVector inverse(const SpatialForceVector & op);

        double dot(const SpatialMotionVector & other) const;

        /** constructor helpers */
        static SpatialForceVector Zero();
    };
}

#endif /* IDYNTREE_SPATIAL_FORCE_RAW_H */