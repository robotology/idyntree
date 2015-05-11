/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_WRENCH_H
#define IDYNTREE_POSITION_H

#include "PositionRaw.h"
#include "PositionSemantics.h"

#include <string>

namespace iDynTree
{
    /**
     * Class representing wrench coordinates.
     * Currenly missing the semantics. 
     *
     */
    class Position: public PositionRaw
    {
    private:
        PositionSemantics semantics;

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

        const Position & changePoint(const Position & newPoint);
        const Position & changeRefPoint(const Position & newRefPoint);
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
        ///@}

    };
}

#endif