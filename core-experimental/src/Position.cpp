/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "Position.h"
#include "Rotation.h"
#include "Utils.h"
#include <cassert>
#include <iostream>
#include <sstream>

namespace iDynTree
{
    // For all the constructors and functions below, checking the semantics while debugging
    // should always be done before the actual composition.

    Position::Position(): PositionRaw()
    {
    }

    Position::Position(double x, double y, double z): PositionRaw(x,y,z)
    {
    }

    Position::Position(const Position & other): PositionRaw(other)
    {
        this->semantics = other.getSemantics();
    }

    Position::Position(const PositionRaw& other): PositionRaw(other)
    {

    }
    
    Position::Position(const PositionRaw & otherPos, const PositionSemantics & otherSem): PositionRaw(otherPos)
    {
        this->semantics = otherSem;
    }

    Position::~Position()
    {
    }

    PositionSemantics& Position::getSemantics()
    {
        return this->semantics;
    }

    const PositionSemantics& Position::getSemantics() const
    {
        return this->semantics;
    }


    const Position& Position::changePoint(const Position& newPoint)
    {
        iDynTreeAssert( this->semantics.changePoint(newPoint.semantics) );
        this->PositionRaw::changePoint(newPoint);
        return *this;
    }

    const Position& Position::changeRefPoint(const Position& newRefPoint)
    {
        iDynTreeAssert( this->semantics.changeRefPoint(newRefPoint.semantics) );
        this->PositionRaw::changeRefPoint(newRefPoint);
        return *this;
    }

    const Position& Position::changeCoordinateFrame(const Rotation & newCoordinateFrame)
    {
        *this = newCoordinateFrame.convertToNewCoordFrame(*this);
        return *this;
    }

    Position Position::compose(const Position& op1, const Position& op2)
    {
        PositionSemantics resultSemantics;
        iDynTreeAssert( PositionSemantics::compose(op1.semantics,op2.semantics,resultSemantics) );
        return Position(PositionRaw::compose(op1,op2),resultSemantics);
    }


    Position Position::inverse(const Position& op)
    {
        PositionSemantics resultSemantics;
        iDynTreeAssert( PositionSemantics::inverse(op.semantics,resultSemantics) );
        return Position(PositionRaw::inverse(op),resultSemantics);
    }

    // overloaded operators
    Position Position::operator+(const Position& other) const
    {
        return compose(*this,other);
    }

    Position Position::operator-(const Position& other) const
    {
        return compose(*this,inverse(other));
    }

    Position Position::operator-() const
    {
        return inverse(*this);
    }
    
    std::string Position::toString() const
    {
        std::stringstream ss;

        ss << PositionRaw::toString() << " " << semantics.toString();

        return ss.str();
    }

    std::string Position::reservedToString() const
    {
        return this->toString();
    }

}
