/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_POSITION_H
#define IDYNTREE_POSITION_H

#include "PositionRaw.h"
#include "PositionSemantics.h"
#include "Rotation.h"

#include <string>

namespace iDynTree
{
    class Rotation;
    
    /**
     * Class representation the coordinates of the Position of
     * a point with respect to another point.
     *
     * \ingroup iDynTreeCore
     *
     * The exact semantics for this class are the one defined as PositionCoord in:
     *
     * De Laet T, Bellens S, Smits R, Aertbeliën E, Bruyninckx H, and De Schutter J
     * (2013), Geometric Relations between Rigid Bodies: Semantics for Standardization,
     * IEEE Robotics & Automation Magazine, Vol. 20, No. 1, pp. 84-93.
     * URL : http://people.mech.kuleuven.be/~tdelaet/geometric_relations_semantics/geometric_relations_semantics_theory.pdf
     *
     * One operation is not included for a logic paradox:
     *   Position(a|A,c|C) = compose(Position(b|B,c|C),Position(a|A,b|B)) is forbidded in iDynTree to avoid ambiguity on compose(Position(b|B,a|A),Position(a|A,b|B))
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
         * Denstructor
         */
        virtual ~Position();
        
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
        
        Position operator+(const Position &other) const;
        Position operator-(const Position &other) const;
        Position operator-() const;
        
        /** @name Output helpers.
         *  Output helpers.
         */
        ///@{
        std::string toString() const;
        
        std::string reservedToString() const;
        ///@}
        
        friend Position Rotation::convertToNewCoordFrame(const Position & op) const;
    };
}

#endif