/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_POSITION_H
#define IDYNTREE_POSITION_H

#include <iDynTree/Core/PositionRaw.h>
#include <iDynTree/Core/PositionSemantics.h>
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
    private:
        PositionSemantics semantics;

        /**
         * Copy constructor: create a Position from a PositionRaw and a PositionSemantics object.
         */
        Position(const PositionRaw & otherPos, const PositionSemantics & otherSem);

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
         * Semantic getter
         */
        PositionSemantics& getSemantics();

        /**
         * Const Semantic getter
         */
        const PositionSemantics& getSemantics() const;

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

       // TODO \todo this information is interesting, but distracting for the average user.
   //             Move it to a design document describing the semantic model of iDynTree .
   //  The exact semantics for this class are inspired to the one define as PositionCoord in:
   //
   //  De Laet T, Bellens S, Smits R, Aertbeliën E, Bruyninckx H, and De Schutter J
   //  (2013), Geometric Relations between Rigid Bodies: Semantics for Standardization,
   //  IEEE Robotics & Automation Magazine, Vol. 20, No. 1, pp. 84-93.
   //  URL : http://people.mech.kuleuven.be/~tdelaet/geometric_relations_semantics/geometric_relations_semantics_theory.pdf
   //
   //  One operation is not mentione in that paper  included for a logic paradox:
   //    Position(a|A,c|C) = compose(Position(b|B,c|C),Position(a|A,b|B)) is forbidded in iDynTree to avoid ambiguity on compose(Position(b|B,a|A),Position(a|A,b|B))
   //
   //
}

#endif
