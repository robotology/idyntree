/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "RotationSemantics.h"
#include "PositionSemantics.h"
#include "Utils.h"
#include <cassert>
#include <iostream>
#include <sstream>

namespace iDynTree
{
    RotationSemantics::RotationSemantics(): orientationFrame(UNKNOWN),
                                            refOrientationFrame(UNKNOWN)
    {

    }

    RotationSemantics::RotationSemantics(int _orientationFrame, int _refOrientationFrame): orientationFrame(_orientationFrame),
                                                                                           refOrientationFrame(_refOrientationFrame)
    {
    }


    RotationSemantics::RotationSemantics(const RotationSemantics& other)
    {
        this->orientationFrame = other.orientationFrame;
        this->refOrientationFrame = other.refOrientationFrame;
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

    void RotationSemantics::setOrientationFrame(int _orientationFrame)
    {
        this->orientationFrame = _orientationFrame;
    }

    void RotationSemantics::setReferenceOrientationFrame(int _refOrientationFrame)
    {
        this->refOrientationFrame = _refOrientationFrame;
    }


    bool RotationSemantics::check_changeOrientFrame(const RotationSemantics& newOrientFrame)
    {
        // check semantics
        if( !checkEqualOrUnknown(this->orientationFrame,newOrientFrame.getReferenceOrientationFrame()) )
        {
            std::cerr << "[ERROR] RotationSemantics::changeOrientFrame : the orientationFrame of this object does not match the referenceOrientationFrame of the newOrientFrame\n";
            assert(false);
            return false;
        }

        return true;
    }

    bool RotationSemantics::check_changeRefOrientFrame(const RotationSemantics& newRefOrientFrame)
    {
        // check semantics
        if( !checkEqualOrUnknown(newRefOrientFrame.getOrientationFrame(),this->refOrientationFrame) )
        {
            std::cerr << "[ERROR] RotationSemantics::changeRefOrientFrame : the refOrientationFrame of this object does not match the orientationFrame of the newRefOrientFrame\n";
            assert(false);
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
            assert(false);
            return false;
        }

        return true;
    }

    bool RotationSemantics::check_inverse2(const RotationSemantics& op)
    {
        return true;
    }

    bool RotationSemantics::check_apply(const RotationSemantics& op1, const PositionSemantics& op2)
    {
        // check semantics
        if( !checkEqualOrUnknown(op1.getOrientationFrame(),op2.getCoordinateFrame() ) )
        {
            std::cerr << "[ERROR] RotationSemantics::apply : the orientationFrame of the Rotation does not match the coordinateFrame of the position\n";
            assert(false);
            return false;
        }

        return true;
    }


    const RotationSemantics& RotationSemantics::changeOrientFrame(const RotationSemantics& newOrientFrame)
    {
        this->orientationFrame = newOrientFrame.getOrientationFrame();

        return *this;
    }

    const RotationSemantics& RotationSemantics::changeRefOrientFrame(const RotationSemantics& newRefOrientFrame)
    {
        this->refOrientationFrame = newRefOrientFrame.getReferenceOrientationFrame();

        return *this;
    }

    void RotationSemantics::compose(const RotationSemantics& op1, const RotationSemantics& op2, RotationSemantics& result)
    {
        result.refOrientationFrame = op1.getReferenceOrientationFrame();
        result.orientationFrame    = op2.getOrientationFrame();
    }

    RotationSemantics RotationSemantics::compose(const RotationSemantics& op1, const RotationSemantics& op2)
    {
        RotationSemantics result;

        compose(op1,op2,result);

        return result;
    }

    void RotationSemantics::inverse2(const RotationSemantics& op, RotationSemantics& result)
    {
        result.refOrientationFrame = op.getOrientationFrame();
        result.orientationFrame    = op.getReferenceOrientationFrame();
    }

    RotationSemantics RotationSemantics::inverse2(const RotationSemantics& op)
    {
        RotationSemantics result;

        RotationSemantics::inverse2(op,result);

        return result;
    }

    void RotationSemantics::apply(const RotationSemantics& op1, const PositionSemantics& op2, PositionSemantics& result)
    {
        result.setCoordinateFrame(op1.getReferenceOrientationFrame());
        result.setPoint(op2.getPoint());
        result.setReferencePoint(op2.getReferencePoint());
    }

    PositionSemantics RotationSemantics::apply(const RotationSemantics& op1, const PositionSemantics& op2)
    {
        PositionSemantics result;

        RotationSemantics::apply(op1,op2,result);

        return result;
    }

    std::string RotationSemantics::toString() const
    {
        std::stringstream ss;

        ss << " Semantics:"
           << " orientationFrame " << this->getOrientationFrame()
           << " referenceOrientationFrame " << this->getReferenceOrientationFrame();

        return ss.str();
    }


}