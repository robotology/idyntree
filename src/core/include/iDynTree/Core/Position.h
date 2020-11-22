/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_POSITION_H
#define IDYNTREE_POSITION_H

#include <iDynTree/Core/PositionRaw.h>
#include <iDynTree/Core/Rotation.h>

#include <string>

namespace iDynTree
{
    class Rotation;
    class Twist;
    class SpatialAcc;
    class SpatialMomentum;
    class Wrench;
    class SpatialMotionVector;
    class SpatialForceVector;

    /**
     * Class representation the coordinates of the Position of
     * a point with respect to another point.
     *
     * \ingroup iDynTreeCore
     *
     * \image html position.svg
     *
     * The Position object can briefly described as the position
     * of a *point* with respect to a *refPoint*, expressed with
     * respect to an orientation given by *orientFrame* .
     *
     */
    class Position: public PositionRaw
    {
    public:
        /**
         * Default constructor: initialize all the coordinates to 0
         */
        Position();

        /**
         * Constructor from 3 doubles: initialize the coordinates with the passed values.
         */
        Position(double x, double y, double z);

        /**
         * Copy constructor: create a Position from another Position
         */
        Position(const Position & other);

        /**
         * Copy constructor: create a Position from a PositionRaw
         */
        Position(const PositionRaw & other);

        /**
         * Create a Position from a span
         */
        Position(iDynTree::Span<const double> other);

        /**
         * Geometric operations
         */
        const Position & changePoint(const Position & newPoint);
        const Position & changeRefPoint(const Position & newRefPoint);
        const Position & changeCoordinateFrame(const Rotation & newCoordinateFrame);
        static Position compose(const Position & op1, const Position & op2);
        static Position inverse(const Position & op);
        SpatialMotionVector  changePointOf(const SpatialMotionVector & other) const;
        SpatialForceVector   changePointOf(const SpatialForceVector  & other) const;
        Twist  changePointOf(const Twist & other) const;
        SpatialAcc      changePointOf(const SpatialAcc & other) const;
        SpatialMomentum changePointOf(const SpatialMomentum & other) const;
        Wrench changePointOf(const Wrench & other) const;

        /**
         * overloaded operators
         */
        Position operator+(const Position &other) const;
        Position operator-(const Position &other) const;
        Position operator-() const;
        Twist    operator*(const Twist    & other) const;
        SpatialForceVector operator*(const SpatialForceVector & other) const;
        SpatialAcc      operator*(const SpatialAcc & other) const;
        SpatialMomentum operator*(const SpatialMomentum & other) const;
        Wrench   operator*(const Wrench   & other) const;

        /** @name Output helpers.
         *  Output helpers.
         */
        ///@{
        std::string toString() const;

        std::string reservedToString() const;
        ///@}

        friend Position Rotation::changeCoordFrameOf(const Position & op) const;

        static Position Zero();
    };
}

#endif
