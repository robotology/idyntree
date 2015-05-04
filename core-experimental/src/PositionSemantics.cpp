/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "PositionSemantics.h"
#include "Utils.h"
#include <cassert>
#include <cstdio>
#include <sstream>

namespace iDynTree
{
    PositionSemantics::PositionSemantics(): point(UNKNOWN),
                                            refPoint(UNKNOWN),
                                            coordinateFrame(UNKNOWN)
    {

    }


    PositionSemantics::PositionSemantics(int _point, int _refPoint, int _coordinateFrame): point(_point),
                                                                                           refPoint(_refPoint),
                                                                                           coordinateFrame(_coordinateFrame)
    {
    }


    PositionSemantics::PositionSemantics(const PositionSemantics& other)
    {
        this->point = other.point;
        this->refPoint = other.refPoint;
        this->coordinateFrame = other.coordinateFrame;
    }

    PositionSemantics::~PositionSemantics()
    {

    }


    void PositionSemantics::setPoint(int _point)
    {
        this->point = _point;
    }

    int PositionSemantics::getPoint() const
    {
        return this->point;
    }

    void PositionSemantics::setReferencePoint(int _referencePoint)
    {
        this->refPoint = _referencePoint;
    }

    int PositionSemantics::getReferencePoint() const
    {
        return this->refPoint;
    }

    void PositionSemantics::setCoordinateFrame(int _coordinateFrame)
    {
        this->coordinateFrame = _coordinateFrame;
    }

    int PositionSemantics::getCoordinateFrame() const
    {
        return this->coordinateFrame;
    }

    bool PositionSemantics::check_changePoint(const PositionSemantics& newPoint)
    {
        // check semantics
        if( !checkEqualOrUnknown(newPoint.coordinateFrame,this->coordinateFrame) )
        {
            fprintf(stderr,"[ERROR] Position::changePoint error: changePoint with newPosition expressed in a different coordinateFrames\n");
            assert(false);
            return false;
        }

        if( !checkEqualOrUnknown(newPoint.refPoint,this->point) )
        {
            fprintf(stderr,"[ERROR] Position::changePoint error: newPosition has a reference point different from the original point\n");
            assert(false);
            return false;
        }

        return true;
    }

    const PositionSemantics& PositionSemantics::changePoint(const PositionSemantics& newPoint)
    {
        // set new semantics
        this->point = newPoint.getPoint();
        return *this;
    }


    bool PositionSemantics::check_changeRefPoint(const PositionSemantics& newPosition)
    {
        // check semantics
        if( !checkEqualOrUnknown(newPosition.coordinateFrame,this->coordinateFrame) )
        {
            fprintf(stderr,"[ERROR] Position::changeRefPoint error: changePoint with newPosition expressed in a different coordinateFrames\n");
            assert(false);
            return false;
        }

        if( !checkEqualOrUnknown(newPosition.point,this->refPoint) )
        {
            fprintf(stderr,"[ERROR] Position::changeRefPoint error: newPosition has a refernce point different from the original point\n");
            assert(false);
            return false;
        }

        return true;
    }

    const PositionSemantics& PositionSemantics::changeRefPoint(const PositionSemantics& newPosition)
    {
        // set new semantics
        this->refPoint = newPosition.refPoint;
        return *this;
    }


    bool PositionSemantics::check_compose(const PositionSemantics& op1, const PositionSemantics& op2)
    {
        // check semantics
        if( !checkEqualOrUnknown(op1.coordinateFrame,op2.coordinateFrame) )
        {
            fprintf(stderr,"[ERROR] Position::compose error: composing two position expressed in different coordinateFrames\n");
            assert(false);
            return false;
        }

        if( !checkEqualOrUnknown(op1.refPoint,op2.point) )
        {
            fprintf(stderr,"[ERROR] Position::compose error: composing two position where the reference point of the first one is different from the point of the second\n");
            if( op1.point == op2.refPoint )
            {
                fprintf(stderr,"[ERROR] Position(a|A,c|C) = compose(Position(b|B,c|C),Position(a|A,b|B)) is forbidded in iDynTree to avoid ambiguity on compose(Position(b|B,a|A),Position(a|A,b|B))\n");
                assert(false);
                return false;
            }
        }

        return true;
    }

    void PositionSemantics::compose(const PositionSemantics& op1, const PositionSemantics& op2, PositionSemantics& result)
    {
        // set new semantics
        result.refPoint = op2.refPoint;
        result.point = op1.point;
        result.coordinateFrame = op1.coordinateFrame;

        return;
    }

    PositionSemantics PositionSemantics::compose(const PositionSemantics& op1, const PositionSemantics& op2)
    {
        PositionSemantics result;
        compose(op1,op2,result);
        return result;
    }

    bool PositionSemantics::check_inverse(const PositionSemantics& op)
    {
        return true;
    }

    void PositionSemantics::inverse(const PositionSemantics& op, PositionSemantics& result)
    {
        result.coordinateFrame = op.coordinateFrame;
        result.point = op.refPoint;
        result.refPoint = op.point;

        return;
    }

    PositionSemantics PositionSemantics::inverse(const PositionSemantics& op)
    {
        PositionSemantics result;
        inverse(op,result);
        return result;
    }

    std::string PositionSemantics::toString() const
    {
        std::stringstream ss;

        ss << " Semantics:"
           << " point " << this->getPoint()
           << " referencePoint " << this->getReferencePoint()
           << " coordinateFrame " << this->getCoordinateFrame();

        return ss.str();
    }




}