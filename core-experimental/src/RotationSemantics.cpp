/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "RotationSemantics.h"
#include "PositionSemantics.h"
#include "Utils.h"
#include <iostream>
#include <sstream>

namespace iDynTree
{
    RotationSemantics::RotationSemantics():
    orientationFrame(UNKNOWN),
    body(UNKNOWN),
    refOrientationFrame(UNKNOWN),
    refBody(UNKNOWN),
    coordinateFrame(UNKNOWN)
    {

    }

    RotationSemantics::RotationSemantics(int _orientationFrame, int _body,
                                         int _refOrientationFrame, int _refBody):
    orientationFrame(_orientationFrame),
    body(_body),
    refOrientationFrame(_refOrientationFrame),
    refBody(_refBody),
    coordinateFrame(_refOrientationFrame)
    {
    }


    RotationSemantics::RotationSemantics(const RotationSemantics& other)
    {
        this->setOrientationFrame(other.getOrientationFrame());
        this->setBody(other.getBody());
        this->setReferenceOrientationFrame(other.getReferenceOrientationFrame());
        this->setRefBody(other.getRefBody());
        this->setCoordinateFrame(other.getCoordinateFrame());
    }

    RotationSemantics::~RotationSemantics()
    {
    }

    int RotationSemantics::getOrientationFrame() const
    {
        return this->orientationFrame;
    }

    int RotationSemantics::getBody() const
    {
        return this->body;
    }
    
    int RotationSemantics::getReferenceOrientationFrame() const
    {
        return this->refOrientationFrame;
    }

    int RotationSemantics::getRefBody() const
    {
        return this->refBody;
    }
    
    int RotationSemantics::getCoordinateFrame() const
    {
        return this->coordinateFrame;
    }
    
    void RotationSemantics::setOrientationFrame(int _orientationFrame)
    {
        this->orientationFrame = _orientationFrame;
    }

    void RotationSemantics::setBody(int _body)
    {
        this->body = _body;
    }
    
    void RotationSemantics::setReferenceOrientationFrame(int _refOrientationFrame)
    {
        this->refOrientationFrame = this->coordinateFrame = _refOrientationFrame;
    }
    
    void RotationSemantics::setRefBody(int _refBody)
    {
        this->refBody = _refBody;
    }
    
    void RotationSemantics::setCoordinateFrame(int _coordinateFrame)
    {
        this->refOrientationFrame = this->coordinateFrame = _coordinateFrame;
    }
    
    bool RotationSemantics::check_changeOrientFrame(const RotationSemantics& newOrientFrame)
    {
        return (   reportErrorIf(!checkEqualOrUnknown(this->orientationFrame,newOrientFrame.refOrientationFrame),
                                 __PRETTY_FUNCTION__,
                                 "the orientationFrame of this object does not match the referenceOrientationFrame of newOrientFrame\n")
                && reportErrorIf(!checkEqualOrUnknown(this->body,newOrientFrame.refBody),
                                 __PRETTY_FUNCTION__,
                                 "the orientationFrame of this object and the referenceOrientationFrame of newOrientFrame are not fixed to the same body\n")
                && reportErrorIf(!checkEqualOrUnknown(newOrientFrame.refBody,newOrientFrame.refBody),
                                 __PRETTY_FUNCTION__,
                                 "the orientationFrame and the referenceOrientationFrame of newOrientFrame are not fixed to the same body\n"));
    }

    bool RotationSemantics::check_changeRefOrientFrame(const RotationSemantics& newRefOrientFrame)
    {
        return (   reportErrorIf(!checkEqualOrUnknown(newRefOrientFrame.orientationFrame,this->refOrientationFrame),
                                 __PRETTY_FUNCTION__,
                                 "the refOrientationFrame of this object does not match the orientationFrame of newRefOrientFrame\n")
                && reportErrorIf(!checkEqualOrUnknown(newRefOrientFrame.body,this->refBody),
                                 __PRETTY_FUNCTION__,
                                 "the orientationFrame and the referenceOrientationFrame of newRefOrientFrame are not fixed to the same body\n")
                && reportErrorIf(!checkEqualOrUnknown(newRefOrientFrame.refBody,newRefOrientFrame.body),
                                 __PRETTY_FUNCTION__,
                                 "the orientationFrame and the referenceOrientationFrame of newRefOrientFrame are not fixed to the same body\n"));
    }

    bool RotationSemantics::check_changeCoordFrameOf(const PositionSemantics & op) const
    {
        return (   reportErrorIf(!checkEqualOrUnknown(this->orientationFrame,op.getCoordinateFrame()),
                                 __PRETTY_FUNCTION__,
                                 "transformation's orientationFrame is different from current Position's coordinateFrame\n")
                && reportErrorIf(!checkEqualOrUnknown(this->body,op.getRefBody()),
                                 __PRETTY_FUNCTION__,
                                 "transformation's orientationFrame and the coordinateFrame of current Position are not fixed to the same body\n")
                && reportErrorIf(!checkEqualOrUnknown(op.getBody(),op.getRefBody()),
                                 __PRETTY_FUNCTION__,
                                 "the transformation's body and reference body should be the same\n"));
    }

    bool RotationSemantics::check_compose(const RotationSemantics& op1, const RotationSemantics& op2)
    {
        return (   reportErrorIf(!checkEqualOrUnknown(op1.orientationFrame,op2.refOrientationFrame),
                                 __PRETTY_FUNCTION__,
                                 "the orientationFrame of the first operand does not match the referenceOrientationFrame of the second operand\n")
                && reportErrorIf(!checkEqualOrUnknown(op1.body,op2.refBody),
                                 __PRETTY_FUNCTION__,
                                 "the body of the first operand does not match the refBody of the second operand\n"));
    }

    bool RotationSemantics::check_inverse2(const RotationSemantics& op)
    {
        return true;
    }

    bool RotationSemantics::changeOrientFrame(const RotationSemantics& newOrientFrame)
    {
        // check semantics
        bool status = this->check_changeOrientFrame(newOrientFrame);
        
        // set new semantics
        this->setOrientationFrame(newOrientFrame.getOrientationFrame());

        return status;
    }

    bool RotationSemantics::changeRefOrientFrame(const RotationSemantics& newRefOrientFrame)
    {
        // check semantics
        bool status = this->check_changeRefOrientFrame(newRefOrientFrame);
        
        // set new semantics
        this->setReferenceOrientationFrame(newRefOrientFrame.getReferenceOrientationFrame());
        
        return status;
    }

    bool RotationSemantics::changeCoordFrameOf(const PositionSemantics & other, PositionSemantics & result) const
    {
        // check semantics
        bool status = this->check_changeCoordFrameOf(other);
        
        // set new semantics
        result = other;
        result.setCoordinateFrame(this->getCoordinateFrame());
        
        return status;
    }
    
    bool RotationSemantics::compose(const RotationSemantics& op1, const RotationSemantics& op2, RotationSemantics& result)
    {
        // check semantics
        bool status = RotationSemantics::check_compose(op1, op2);
        
        // set new semantics
        result.setOrientationFrame(op2.getOrientationFrame());
        result.setBody(op2.getBody());
        result.setReferenceOrientationFrame(op1.getReferenceOrientationFrame());
        result.setRefBody(op1.getRefBody());
        
        return status;
    }

    bool RotationSemantics::inverse2(const RotationSemantics& op, RotationSemantics& result)
    {
        // check semantics
        bool status = RotationSemantics::check_inverse2(op);
        
        result.setOrientationFrame(op.getReferenceOrientationFrame());
        result.setBody(op.getRefBody());
        result.setReferenceOrientationFrame(op.getOrientationFrame());
        result.setRefBody(op.getBody());
        
        return status;
    }

    std::string RotationSemantics::toString() const
    {
        std::stringstream ss;

        ss << " Semantics:"
           << " orientationFrame " << this->getOrientationFrame()
           << " body " << this->getBody()
           << " referenceOrientationFrame " << this->getReferenceOrientationFrame()
           << " reference body " << this->getRefBody();

        return ss.str();
    }

    std::string RotationSemantics::reservedToString() const
    {
        return this->toString();
    }

}