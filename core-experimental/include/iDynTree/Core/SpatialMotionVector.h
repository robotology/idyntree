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
//#include <iDynTree/Core/ISpatialVector.h>

namespace iDynTree
{
    class PositionRaw;
    class RotationRaw;
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
    class SpatialMotionVector: public Vector6
    {
    protected:
        //LinearMotionVector3 linearVec3;
        //AngularMotionVector3 angularVec3;
        //SpatialMotionVectorSemantics semantics;
        
    public:
        /**
         * constructors
         */
        SpatialMotionVector();
        SpatialMotionVector(const double* in_data, const unsigned int in_size);
        SpatialMotionVector(const SpatialMotionVector & other);
        virtual ~SpatialMotionVector();

        /**
         * Geometric operations
         */
        const SpatialMotionVector & changePoint(const PositionRaw & newPoint);
        const SpatialMotionVector & changeCoordFrame(const RotationRaw & newCoordFrame);
        static SpatialMotionVector compose(const SpatialMotionVector & op1, const SpatialMotionVector & op2);
        static SpatialMotionVector inverse(const SpatialMotionVector & op);

        double dot(const SpatialForceVector& other) const;

        /**
         * Cross products
         */
        SpatialMotionVector cross(const SpatialMotionVector& other) const;
        SpatialForceVector cross(const SpatialForceVector& other) const;

        /** overloaded operators **/;

        /** constructor helpers */
        static SpatialMotionVector Zero();

        /** @name Output helpers.
         *  Output helpers.
         */
        ///@{
        //std::string toString() const;
        
        //std::string reservedToString() const;
        ///@}
    };
}

#endif /* IDYNTREE_SPATIAL_MOTION_RAW_H */