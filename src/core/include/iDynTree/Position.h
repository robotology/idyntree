// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_POSITION_H
#define IDYNTREE_POSITION_H

#include <iDynTree/Rotation.h>

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
    class Position: public Vector3
    {
    public:
        /**
         * Default constructor.
         * The data is not initialized, please initialize the data in the created object before use.
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
         * Assignment operator: assign a Position from another Position
         */
        Position& operator=(const Position& other);

        /**
         * Constructor from a raw buffer of 3 doubles.
         */
        Position(const double* in_data, const unsigned int in_size);

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

    IDYNTREE_DEPRECATED_WITH_MSG("iDynTree::PositionRaw is deprecated, use iDynTree::Position") typedef Position PositionRaw;

}

#endif
