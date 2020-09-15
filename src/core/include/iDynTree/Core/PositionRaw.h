/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_POSITION_RAW_H
#define IDYNTREE_POSITION_RAW_H


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
         * Default constructor.
         * The data is not reset to 0 for perfomance reason.
         * Please initialize the data in the vector before any use.
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
         * Construct from a span
         * @warning if the Span size is different from 3 an assert is thrown at run-time.
         */
        PositionRaw(iDynTree::Span<const double> other);

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
