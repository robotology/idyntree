/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "TransformSemantics.h"
#include "RotationSemantics.h"
#include "PositionSemantics.h"
#include "Utils.h"
#include <cassert>
#include <iostream>
#include <sstream>

namespace iDynTree
{
    
    TransformSemantics::TransformSemantics(): point(UNKNOWN),
                                              orientationFrame(UNKNOWN),
                                              refPoint(UNKNOWN),
                                              refOrientationFrame(UNKNOWN)
    {
        
    }
    
    TransformSemantics::TransformSemantics(int _orientationFrame, int _refOrientationFrame): point(_orientationFrame),
                                                                                             orientationFrame(_orientationFrame),
                                                                                             refPoint(_refOrientationFrame),
                                                                                             refOrientationFrame(_refOrientationFrame),
                                                                                             coordinateFrame(_refOrientationFrame)
    {
        
    }
    
    TransformSemantics::TransformSemantics(const TransformSemantics& other): point(other.point),
                                                                             orientationFrame(other.orientationFrame),
                                                                             refPoint(other.refPoint),
                                                                             refOrientationFrame(other.refOrientationFrame),
                                                                             coordinateFrame(other.coordinateFrame)
    {
        
    }
    
    TransformSemantics::~TransformSemantics()
    {
        
    }
    
    int TransformSemantics::getPoint() const
    {
        return point;
    }
    
    int TransformSemantics::getOrientationFrame() const
    {
        return orientationFrame;
    }
    
    int TransformSemantics::getReferencePoint() const
    {
        return refPoint;
    }
    
    int TransformSemantics::getReferenceOrientationFrame() const
    {
        return refOrientationFrame;
    }
    
    int TransformSemantics::getCoordinateFrame() const
    {
        return coordinateFrame;
    }
    
    void TransformSemantics::setPoint(int _point)
    {
        this->point = _point;
    }
    
    void TransformSemantics::setOrientationFrame(int _orientationFrame)
    {
        this->orientationFrame = _orientationFrame;
    }
    
    void TransformSemantics::setReferencePoint(int _refPoint)
    {
        this->refPoint = _refPoint;
    }
    
    void TransformSemantics::setReferenceOrientationFrame(int _refOrientationFrame)
    {
        this->refOrientationFrame = _refOrientationFrame;
    }
    
    const PositionSemantics TransformSemantics::getPositionSemantics() const
    {
        return PositionSemantics(this->getPoint(),
                                 this->getReferencePoint(),
                                 this->getReferenceOrientationFrame());
    }
    
    const RotationSemantics TransformSemantics::getRotationSemantics() const
    {
        return RotationSemantics(this->getOrientationFrame(),
                                 this->getReferenceOrientationFrame());
    }
    
    bool TransformSemantics::setPositionSemantics(const PositionSemantics& position)
    {
        // check consistency of setted position with existing rotation
        // \todo TODO should be move to a proper check_* method ?
        if( !checkEqualOrUnknown(position.getCoordinateFrame(),this->getReferenceOrientationFrame()) )
        {
            reportError("TransformSemantics",
                        "setPositionSemantics",
                        "Mismatch between coordinateFrame of the setted position and referenceOrientationFrame of the Transform");
            
            return false;
        }
        
        this->point = position.getPoint();
        this->refPoint = position.getReferencePoint();
        if( this->getReferenceOrientationFrame() == UNKNOWN )
        {
            this->refOrientationFrame = position.getCoordinateFrame();
        }
        
        return true;
    }
    
    bool TransformSemantics::setRotationSemantics(const RotationSemantics& rotation)
    {
        // check consistency of setted rotation with existing position
        // \todo TODO should be move to a proper check_* method ?
        if( !checkEqualOrUnknown(rotation.getReferenceOrientationFrame(),this->getReferenceOrientationFrame()) )
        {
            reportError("TransformSemantics",
                        "setRotationSemantics",
                        "Mismatch between coordinateFrame of the setted orientation and referenceOrientationFrame of the Transform");
            
            return false;
        }
        
        this->orientationFrame = rotation.getOrientationFrame();
        if( this->getReferenceOrientationFrame() == UNKNOWN )
        {
            this->refOrientationFrame = rotation.getReferenceOrientationFrame();
        }
        
        return true;
    }
    
    bool TransformSemantics::check_compose(const TransformSemantics& op1, const TransformSemantics& op2)
    {
        if( !checkEqualOrUnknown(op1.getOrientationFrame(),op2.getReferenceOrientationFrame()) )
        {
            reportError("TransformSemantics",
                        "compose",
                        "Mismatch between orientationFrame of first operand and referenceOrientationFrame of second operand");
            
            return false;
        }
        
        if( !checkEqualOrUnknown(op1.getPoint(),op2.getReferencePoint()) )
        {
            reportError("TransformSemantics",
                        "compose",
                        "Mismatch between point of first operand and referencePoint of second operand");
            
            return false;
        }
        
        return true;
    }
    
    bool TransformSemantics::check_inverse2(const TransformSemantics& orient)
    {
        return true;
    }
    
    bool TransformSemantics::check_transform(const TransformSemantics& op1, const PositionSemantics& op2)
    {
        if( !checkEqualOrUnknown(op1.getOrientationFrame(),op2.getCoordinateFrame()) )
        {
            reportError("TransformSemantics",
                        "apply",
                        "Mismatch between orientationFrame of first operand and coordinateFrame of second operand");
            
            return false;
        }
        
        if( !checkEqualOrUnknown(op1.getPoint(),op2.getReferencePoint()) )
        {
            reportError("TransformSemantics",
                        "compose",
                        "Mismatch between point of first operand and referencePoint of second operand");
            
            return false;
        }
        
        return true;
    }
    
    void TransformSemantics::compose(const TransformSemantics& op1, const TransformSemantics& op2, TransformSemantics& result)
    {
        result.setPoint(op2.getPoint());
        result.setOrientationFrame(op2.getOrientationFrame());
        result.setReferencePoint(op1.getReferencePoint());
        result.setReferenceOrientationFrame(op1.getReferenceOrientationFrame());
    }
    
    TransformSemantics TransformSemantics::compose(const TransformSemantics& op1, const TransformSemantics& op2)
    {
        TransformSemantics result;
        TransformSemantics::compose(op1,op2,result);
        return result;
    }
    
    void TransformSemantics::inverse2(const TransformSemantics& trans, TransformSemantics& result)
    {
        result.setPoint(trans.getReferencePoint());
        result.setOrientationFrame(trans.getReferenceOrientationFrame());
        result.setReferencePoint(trans.getPoint());
        result.setReferenceOrientationFrame(trans.getOrientationFrame());
    }
    
    TransformSemantics TransformSemantics::inverse2(const TransformSemantics& trans)
    {
        TransformSemantics result;
        TransformSemantics::inverse2(trans,result);
        return result;
    }
    
    void TransformSemantics::transform(const TransformSemantics& op1, const PositionSemantics& op2, PositionSemantics& result)
    {
        result.setPoint(op2.getPoint());
        result.setReferencePoint(op1.getReferencePoint());
        result.setCoordinateFrame(op1.getReferenceOrientationFrame());
        
        return;
    }
    
    PositionSemantics TransformSemantics::transform(const TransformSemantics& op1, const PositionSemantics& op2)
    {
        PositionSemantics result;
        transform(op1,op2,result);
        return result;
    }
    
    TransformSemantics TransformSemantics::operator*(const TransformSemantics& other) const
    {
        return compose(*this,other);
    }
    
    TransformSemantics TransformSemantics::inverse() const
    {
        return inverse2(*this);
    }
    
    PositionSemantics TransformSemantics::operator*(const PositionSemantics& op2) const
    {
        return transform(*this,op2);
    }
    
    std::string TransformSemantics::toString() const
    {
        std::stringstream ss;
        
        ss << " Semantics:"
        << " point " << this->getPoint()
        << " orientationFrame " << this->getOrientationFrame()
        << " referencePoint " << this->getReferencePoint()
        << " referenceOrientationFrame " << this->getReferenceOrientationFrame();
        
        return ss.str();
    }
    
    std::string TransformSemantics::reservedToString() const
    {
        return this->toString();
    }
    
    
    
}