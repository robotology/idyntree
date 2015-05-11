/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "Position.h"
#include "Utils.h"
#include <cassert>
#include <iostream>
#include <sstream>

namespace iDynTree
{

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
        if( semantics.check_changePoint(newPoint.semantics) )
        {
            PositionRaw::changePoint(newPoint);
            this->semantics.changePoint(newPoint.semantics);
        }

        return *this;
    }

    const Position& Position::changeRefPoint(const Position& newPosition)
    {
        if( semantics.check_changeRefPoint(newPosition.semantics) )
        {
            PositionRaw::changeRefPoint(newPosition);
            this->semantics.changeRefPoint(newPosition.semantics);
        }

        return *this;
    }

    Position Position::compose(const Position& op1, const Position& op2)
    {
        Position result;

        if( PositionSemantics::check_compose(op1.semantics,op2.semantics) )
        {
            result = PositionRaw::compose(op1,op2);
            PositionSemantics::compose(op1.semantics,op2.semantics,result.semantics);
        }

        return result;
    }


    Position Position::inverse(const Position& op)
    {
        Position result;

        if( PositionSemantics::check_inverse(op.semantics) )
        {
            result = PositionRaw::inverse(op);
            PositionSemantics::inverse(op.semantics,result.semantics);
        }

        return result;

    }

    // overloaded operators
    Position Position::operator+(const Position& other) const
    {
        return compose(*this,other);
    }

    Position Position::operator-() const
    {
        return inverse(*this);
    }

    Position Position::operator-(const Position& other) const
    {
        return compose(*this,inverse(other));
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
