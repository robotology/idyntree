/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_SPATIAL_MOTION_RAW_H
#define IDYNTREE_SPATIAL_MOTION_RAW_H

#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/LinearMotionVector3.h>
#include <iDynTree/Core/AngularMotionVector3.h>
#include <iDynTree/Core/SpatialVector.h>

namespace iDynTree
{
    class SpatialForceVector;

    /**
     * Class providing the raw coordinates for any motion spatial vector.
     *
     * \ingroup iDynTreeCore
     *
     * A motion spatial vector can be used to to describe a  twist, twist acceleration,
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

    class SpatialMotionVector: public SpatialVector<SpatialMotionVector, LinearMotionVector3, AngularMotionVector3>
    {
    public:
        /**
         * constructors
         */
        SpatialMotionVector();
        SpatialMotionVector(const LinearMotionVector3 & _linearVec3, const AngularMotionVector3 & _angularVec3);
        SpatialMotionVector(const SpatialMotionVector & other);
        virtual ~SpatialMotionVector();

        /**
         * Cross products
         */
        SpatialMotionVector cross(const SpatialMotionVector& other) const;
        SpatialForceVector cross(const SpatialForceVector& other) const;
    };

}

#endif /* IDYNTREE_SPATIAL_MOTION_RAW_H */