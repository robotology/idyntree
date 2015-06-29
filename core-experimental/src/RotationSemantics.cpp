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
        if( !checkEqualOrUnknown(this->orientationFrame,newOrientFrame.getReferenceOrientationFrame()) )
        {
            std::cerr << "[ERROR] RotationSemantics::changeOrientFrame : the orientationFrame of this object does not match the referenceOrientationFrame of the newOrientFrame\n";
            return false;
        }

        return true;
    }

    bool RotationSemantics::check_changeRefOrientFrame(const RotationSemantics& newRefOrientFrame)
    {
        if( !checkEqualOrUnknown(newRefOrientFrame.getOrientationFrame(),this->refOrientationFrame) )
        {
            std::cerr << "[ERROR] RotationSemantics::changeRefOrientFrame : the refOrientationFrame of this object does not match the orientationFrame of the newRefOrientFrame\n";
            return false;
        }

        return true;
    }

    bool RotationSemantics::check_convertToNewCoordFrame(const PositionSemantics & op) const
    {
        if( !checkEqualOrUnknown(this->orientationFrame,op.getCoordinateFrame()) )
        {
            fprintf(stderr,"[ERROR] Position::convertToNewCoordFrame error: transformation's orientationFrame is different from current Position's coordinateFrame\n");
            return false;
        }
        
        return true;
    }

    bool RotationSemantics::check_compose(const RotationSemantics& op1, const RotationSemantics& op2)
    {
        // check semantics
        if( !checkEqualOrUnknown(op1.getOrientationFrame(),op2.getReferenceOrientationFrame()) )
        {
            std::cerr << "[ERROR] RotationSemantics::compose : the orientationFrame of the first operand does not match the referenceOrientationFrame of the second operand\n";
            return false;
        }

        return true;
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
        result.refOrientationFrame = op.getOrientationFrame();
        result.orientationFrame    = op.getReferenceOrientationFrame();
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