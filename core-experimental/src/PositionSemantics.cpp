/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "PositionSemantics.h"
#include "RotationSemantics.h"
#include "Utils.h"
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
        return (   reportErrorIf(!checkEqualOrUnknown(newPoint.coordinateFrame,this->coordinateFrame),
                                 __PRETTY_FUNCTION__,
                                 "newPosition expressed in a different coordinateFrames\n")
                && reportErrorIf(!checkEqualOrUnknown(newPoint.refPoint,this->point),
                                 __PRETTY_FUNCTION__,
                                 "newPosition has a reference point different from the original point\n"));
    }
    
    bool PositionSemantics::changePoint(const PositionSemantics& newPoint)
    {
        // check semantics
        bool status = this->check_changePoint(newPoint);
        
        // set new semantics
        this->point = newPoint.getPoint();
        
        return status;
    }
    
    
    bool PositionSemantics::check_changeRefPoint(const PositionSemantics& newPosition)
    {
        return (   reportErrorIf(!checkEqualOrUnknown(newPosition.coordinateFrame,this->coordinateFrame),
                                 __PRETTY_FUNCTION__,
                                 "newPosition expressed in a different coordinateFrames\n")
                && reportErrorIf(!checkEqualOrUnknown(newPosition.point,this->refPoint),
                                 __PRETTY_FUNCTION__,
                                 "newPosition point is different from the original reference point\n"));
    }
    
    bool PositionSemantics::changeRefPoint(const PositionSemantics& newPosition)
    {
        // check semantics
        bool status = this->check_changeRefPoint(newPosition);
        
        // set new semantics
        this->refPoint = newPosition.refPoint;
        
        return status;
    }

    bool PositionSemantics::check_changeCoordinateFrame(const RotationSemantics & newCoordinateFrame)
    {
        return reportErrorIf(!checkEqualOrUnknown(newCoordinateFrame.getOrientationFrame(),this->coordinateFrame),
                             __PRETTY_FUNCTION__,
                             "transformation's orientationFrame is different from current coordinateFrame\n");
    }

    bool PositionSemantics::changeCoordinateFrame(const RotationSemantics & newCoordinateFrame)
    {
        // check semantics
        bool status = this->check_changeCoordinateFrame(newCoordinateFrame);
        
        // set new semantics
        this->coordinateFrame = newCoordinateFrame.getCoordinateFrame();
        
        return status;
    }
    
    bool PositionSemantics::check_compose(const PositionSemantics& op1, const PositionSemantics& op2)
    {
        return (   reportErrorIf(!checkEqualOrUnknown(op1.coordinateFrame,op2.coordinateFrame),
                                 __PRETTY_FUNCTION__,
                                 "composing two position expressed in different coordinateFrames\n")
                && reportErrorIf(!checkEqualOrUnknown(op1.refPoint,op2.point) && (op1.point == op2.refPoint),
                                 __PRETTY_FUNCTION__,
                                 "Position(a|A,c|C) = compose(Position(b|B,c|C),Position(a|A,b|B)) is forbidded in iDynTree to avoid ambiguity on compose(Position(b|B,a|A),Position(a|A,b|B))\n")
                && reportErrorIf(!checkEqualOrUnknown(op1.refPoint,op2.point),
                                 __PRETTY_FUNCTION__,
                                 "composing two position expressed in different coordinateFrames\n"));
    }

    bool PositionSemantics::compose(const PositionSemantics& op1, const PositionSemantics& op2, PositionSemantics& result)
    {
        // check semantics
        bool status = PositionSemantics::check_compose(op1,op2);
        
        // set new semantics
        result.refPoint = op2.refPoint;
        result.point = op1.point;
        result.coordinateFrame = op1.coordinateFrame;
        
        return status;
    }

    bool PositionSemantics::check_inverse(const PositionSemantics& op)
    {
        return true;
    }

    bool PositionSemantics::inverse(const PositionSemantics& op, PositionSemantics& result)
    {
        // check semantics
        bool status = PositionSemantics::check_inverse(op);
        
        // set new semantics
        result.coordinateFrame = op.coordinateFrame;
        result.point = op.refPoint;
        result.refPoint = op.point;
        
        return status;
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

    std::string PositionSemantics::reservedToString() const
    {
        return this->toString();
    }



}