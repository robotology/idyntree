/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_POSITION_RAW_H
#define IDYNTREE_POSITION_RAW_H

#include <iDynTree/Core/IVector.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <string>

namespace iDynTree
{
    class RotationRaw;
    class SpatialMotionVector;
    class SpatialForceVector;
    
    /**
     * Class providing the raw coordinates for iDynTree::Position class.
     *
     * \ingroup iDynTreeCore
     */
    class PositionRaw: public Vector3
    {
    public:
        /**
         * Default constructor: initialize all the coordinates to 0
         */
        PositionRaw();

        /**
         * Constructor from 3 doubles: initialize the coordinates with the passed values.
         */
        PositionRaw(double x, double y, double z);

        /**
         * Constructor from a raw buffer of 3 doubles.
         */
        PositionRaw(const double* in_data, const unsigned int in_size);

        /**
         * Copy constructor: create a PositionRaw from another PositionRaw
         */
        PositionRaw(const PositionRaw & other);

        /**
         * Destructor
         */
        virtual ~PositionRaw();

        /**
         * Geometric operations
         */
        const PositionRaw & changePoint(const PositionRaw & newPoint);
        const PositionRaw & changeRefPoint(const PositionRaw & newRefPoint);
        static PositionRaw compose(const PositionRaw & op1, const PositionRaw & op2);
        static PositionRaw inverse(const PositionRaw & op);
        SpatialMotionVector changePointOf(const SpatialMotionVector & other) const;
        SpatialForceVector changePointOf(const SpatialForceVector & other) const;
        
        
        /** @name Output helpers.
         *  Output helpers.
         */
        ///@{
        std::string toString() const;

        std::string reservedToString() const;
        ///@}

    };

}

#endif /* IDYNTREE_POSITION_RAW_H */
