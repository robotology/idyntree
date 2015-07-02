/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_TRANSFORM_RAW_H
#define IDYNTREE_TRANSFORM_RAW_H

#include <string>
#include "PositionRaw.h"
#include "RotationRaw.h"

namespace iDynTree
{
    class SpatialMotionVectorRaw;
    class SpatialForceVectorRaw;
    class SpatialInertiaRaw;

    /**
     * Class providing the raw coordinates for iDynTree::Transform class.
     * This class matches the specific representation of a pose as an
     * homogeneous transformatin matrix.
     *
     * \ingroup iDynTreeCore
     */
    class TransformRaw
    {
    protected:
        RotationRaw rot;
        PositionRaw pos;

    public:
        /**
         * Default constructor: initialize the rotation to the identity and the translation to 0
         */
        TransformRaw();

        /**
         * Constructor from a rotation and a point
         */
        TransformRaw(const RotationRaw & rot, const PositionRaw & origin);

        /**
         * Copy constructor: create a Transform from a TransformRaw.
         */
        TransformRaw(const TransformRaw & other);

        /**
         * Destructor
         */
        virtual ~TransformRaw();

        /**
         * Geometric operations.
         */
        static TransformRaw compose(const TransformRaw & op1, const TransformRaw & op2);
        static TransformRaw inverse2(const TransformRaw & trans);
        static PositionRaw transform(const TransformRaw & op1, const PositionRaw & op2);
        static SpatialMotionVectorRaw transform(const TransformRaw & op1, const SpatialMotionVectorRaw & op2);
        static SpatialForceVectorRaw  transform(const TransformRaw & op1, const SpatialForceVectorRaw & op2);
        static SpatialInertiaRaw      transform(const TransformRaw & op1, const SpatialInertiaRaw & op2);

        /**
         * constructor helpers
         */
        static TransformRaw Identity();

        /** @name Output helpers.
         *  Output helpers.
         */
        ///@{
        std::string toString() const;

        std::string reservedToString() const;
        ///@}
    };
}

#endif