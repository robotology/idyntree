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
    RotationSemantics::RotationSemantics(): orientationFrame(UNKNOWN),
                                            refOrientationFrame(UNKNOWN),
                                            coordinateFrame(UNKNOWN)
    {

    }

    RotationSemantics::RotationSemantics(int _orientationFrame, int _refOrientationFrame): orientationFrame(_orientationFrame),
                                                                                           refOrientationFrame(_refOrientationFrame),
                                                                                           coordinateFrame(_refOrientationFrame)
    {
    }


    RotationSemantics::RotationSemantics(const RotationSemantics& other)
    {
        this->orientationFrame = other.orientationFrame;
        this->refOrientationFrame = other.refOrientationFrame;
        this->coordinateFrame = other.coordinateFrame;
    }

    RotationSemantics::~RotationSemantics()
    {

    }

    int RotationSemantics::getOrientationFrame() const
    {
        return this->orientationFrame;
    }

    int RotationSemantics::getReferenceOrientationFrame() const
    {
        return this->refOrientationFrame;
    }

    int RotationSemantics::getCoordinateFrame() const
    {
        return this->coordinateFrame;
    }
    
    void RotationSemantics::setOrientationFrame(int _orientationFrame)
    {
        this->orientationFrame = _orientationFrame;
    }

    void RotationSemantics::setReferenceOrientationFrame(int _refOrientationFrame)
    {
        this->refOrientationFrame = _refOrientationFrame;
        this->coordinateFrame = _refOrientationFrame;
    }
    
    bool RotationSemantics::check_changeOrientFrame(const RotationSemantics& newOrientFrame)
    {
        return reportErrorIf(!checkEqualOrUnknown(this->orientationFrame,newOrientFrame.getReferenceOrientationFrame()),
                             __PRETTY_FUNCTION__,
                             "the orientationFrame of this object does not match the referenceOrientationFrame of the newOrientFrame\n");
    }

    bool RotationSemantics::check_changeRefOrientFrame(const RotationSemantics& newRefOrientFrame)
    {
        return reportErrorIf(!checkEqualOrUnknown(newRefOrientFrame.getOrientationFrame(),this->refOrientationFrame),
                             __PRETTY_FUNCTION__,
                             "the refOrientationFrame of this object does not match the orientationFrame of the newRefOrientFrame\n");
    }

    bool RotationSemantics::check_convertToNewCoordFrame(const PositionSemantics & op) const
    {
        return reportErrorIf(!checkEqualOrUnknown(this->orientationFrame,op.getCoordinateFrame()),
                             __PRETTY_FUNCTION__,
                             "transformation's orientationFrame is different from current Position's coordinateFrame\n");
    }

    bool RotationSemantics::check_compose(const RotationSemantics& op1, const RotationSemantics& op2)
    {
        return reportErrorIf(!checkEqualOrUnknown(op1.getOrientationFrame(),op2.getReferenceOrientationFrame()),
                             __PRETTY_FUNCTION__,
                             "the orientationFrame of the first operand does not match the referenceOrientationFrame of the second operand\n");
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
        this->orientationFrame = newOrientFrame.orientationFrame;

        return status;
    }

    bool RotationSemantics::changeRefOrientFrame(const RotationSemantics& newRefOrientFrame)
    {
        // check semantics
        bool status = this->check_changeRefOrientFrame(newRefOrientFrame);
        
        // set new semantics
        this->refOrientationFrame = newRefOrientFrame.refOrientationFrame;
        
        return status;
    }

    bool RotationSemantics::convertToNewCoordFrame(const PositionSemantics & other, PositionSemantics & result) const
    {
        // check semantics
        bool status = this->check_convertToNewCoordFrame(other);
        
        // set new semantics
        result.setCoordinateFrame(this->getCoordinateFrame());
        
        return status;
    }
    
    bool RotationSemantics::compose(const RotationSemantics& op1, const RotationSemantics& op2, RotationSemantics& result)
    {
        // check semantics
        bool status = RotationSemantics::check_compose(op1, op2);
        
        // set new semantics
        result.refOrientationFrame = op1.getReferenceOrientationFrame();
        result.orientationFrame    = op2.getOrientationFrame();
        
        return status;
    }

    bool RotationSemantics::inverse2(const RotationSemantics& op, RotationSemantics& result)
    {
        // check semantics
        bool status = RotationSemantics::check_inverse2(op);
        
        result.refOrientationFrame = op.getOrientationFrame();
        result.orientationFrame    = op.getReferenceOrientationFrame();
        
        return status;
    }

    std::string RotationSemantics::toString() const
    {
        std::stringstream ss;

        ss << " Semantics:"
           << " orientationFrame " << this->getOrientationFrame()
           << " referenceOrientationFrame " << this->getReferenceOrientationFrame();

        return ss.str();
    }

    std::string RotationSemantics::reservedToString() const
    {
        return this->toString();
    }

}