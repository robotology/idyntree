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
                                            body(UNKNOWN),
                                            refPoint(UNKNOWN),
                                            refBody(UNKNOWN),
                                            coordinateFrame(UNKNOWN)
    {
    }


    PositionSemantics::PositionSemantics(int _point, int _body,
                                         int _refPoint, int _refBody,
                                         int _coordinateFrame): point(_point),
                                                                body(_body),
                                                                refPoint(_refPoint),
                                                                refBody(_refBody),
                                                                coordinateFrame(_coordinateFrame)
    {
    }


    PositionSemantics::PositionSemantics(const PositionSemantics& other)
    {
        this->point = other.point;
        this->body = other.body;
        this->refPoint = other.refPoint;
        this->refBody = other.refBody;
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

    void PositionSemantics::setBody(int _body)
    {
        this->body = _body;
    }

    int PositionSemantics::getBody() const
    {
        return this->body;
    }

    void PositionSemantics::setReferencePoint(int _referencePoint)
    {
        this->refPoint = _referencePoint;
    }

    int PositionSemantics::getReferencePoint() const
    {
        return this->refPoint;
    }

    void PositionSemantics::setRefBody(int _refBody)
    {
        this->refBody = _refBody;
    }
    
    int PositionSemantics::getRefBody() const
    {
        return this->refBody;
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
                                 "newPoint expressed in a different coordinateFrame\n")
                && reportErrorIf(!checkEqualOrUnknown(newPoint.refPoint,this->point),
                                 __PRETTY_FUNCTION__,
                                 "newPoint has a reference point different from the original Position point\n")
                && reportErrorIf(!checkEqualOrUnknown(newPoint.body,newPoint.refBody),
                                 __PRETTY_FUNCTION__,
                                 "newPoint point and reference point are not fixed to the same body\n")
                && reportErrorIf(!checkEqualOrUnknown(newPoint.refBody,this->body),
                                 __PRETTY_FUNCTION__,
                                 "newPoint reference point and original Position point are not fixed to the same body\n"));
    }
    
    bool PositionSemantics::changePoint(const PositionSemantics& newPoint)
    {
        // check semantics
        bool status = this->check_changePoint(newPoint);
        
        // set new semantics
        this->point = newPoint.getPoint();
        
        return status;
    }
    
    
    bool PositionSemantics::check_changeRefPoint(const PositionSemantics& newRefPoint)
    {
        return (   reportErrorIf(!checkEqualOrUnknown(newRefPoint.coordinateFrame,this->coordinateFrame),
                                 __PRETTY_FUNCTION__,
                                 "newRefPoint expressed in a different coordinateFrames\n")
                && reportErrorIf(!checkEqualOrUnknown(newRefPoint.point,this->refPoint),
                                 __PRETTY_FUNCTION__,
                                 "newRefPoint point is different from the original reference point\n")
                && reportErrorIf(!checkEqualOrUnknown(newRefPoint.body,newRefPoint.refBody),
                                 __PRETTY_FUNCTION__,
                                 "newRefPoint point and reference point are not fixed to the same body\n")
                && reportErrorIf(!checkEqualOrUnknown(newRefPoint.body,this->refBody),
                                 __PRETTY_FUNCTION__,
                                 "newRefPoint point and original Position reference point are not fixed to the same body\n"));
    }
    
    bool PositionSemantics::changeRefPoint(const PositionSemantics& newRefPoint)
    {
        // check semantics
        bool status = this->check_changeRefPoint(newRefPoint);
        
        // set new semantics
        this->refPoint = newRefPoint.refPoint;
        
        return status;
    }

    bool PositionSemantics::check_compose(const PositionSemantics& op1, const PositionSemantics& op2)
    {
        return (   reportErrorIf(!checkEqualOrUnknown(op1.coordinateFrame,op2.coordinateFrame),
                                 __PRETTY_FUNCTION__,
                                 "composing two positions expressed in different coordinateFrames\n")
                && reportErrorIf(!checkEqualOrUnknown(op1.refPoint,op2.point) && (op1.point == op2.refPoint),
                                 __PRETTY_FUNCTION__,
                                 "Position(a|A,c|C) = compose(Position(b|B,c|C),Position(a|A,b|B)) is forbidded in iDynTree to avoid ambiguity on compose(Position(b|B,a|A),Position(a|A,b|B))\n")
                && reportErrorIf(!checkEqualOrUnknown(op1.refPoint,op2.point),
                                 __PRETTY_FUNCTION__,
                                 "Position op1 reference point and Position op2 point don't match\n")
                && reportErrorIf(!checkEqualOrUnknown(op1.refBody,op2.body),
                                 __PRETTY_FUNCTION__,
                                 "Position op1 reference body and Position op2 body don't match\n"));
    }

    bool PositionSemantics::compose(const PositionSemantics& op1, const PositionSemantics& op2, PositionSemantics& result)
    {
        // check semantics
        bool status = PositionSemantics::check_compose(op1,op2);
        
        // set new semantics
        result.point = op1.point;
        result.body = op1.body;
        result.refPoint = op2.refPoint;
        result.refBody = op2.refBody;
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
        result.point = op.refPoint;
        result.body = op.refBody;
        result.refPoint = op.point;
        result.refBody = op.body;
        result.coordinateFrame = op.coordinateFrame;
        
        return status;
    }

    std::string PositionSemantics::toString() const
    {
        std::stringstream ss;

        ss << " Semantics:"
           << " point " << this->getPoint()
           << " body " << this->getBody()
           << " reference Point " << this->getReferencePoint()
           << " reference body " << this->getRefBody()
           << " coordinate Frame " << this->getCoordinateFrame();

        return ss.str();
    }

    std::string PositionSemantics::reservedToString() const
    {
        return this->toString();
    }



}