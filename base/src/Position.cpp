/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "Position.h"
#include "Utils.h"
#include <cassert>
#include <cstdio>
#include <sstream>

namespace iDynTree
{

    Position::Position()
    {
        this->privateData[0] = this->privateData[1] = this->privateData[2] = 0.0;
        this->point = this->referencePoint = this->coordinateFrame = -1;
    }

    Position::Position(double x, double y, double z)
    {
        this->privateData[0] = x;
        this->privateData[1] = y;
        this->privateData[2] = z;
        this->point = this->referencePoint = this->coordinateFrame = -1;
    }

    Position::Position(const Position & other)
    {
        this->privateData[0] = other[0];
        this->privateData[1] = other[1];
        this->privateData[2] = other[2];
        this->point = this->referencePoint = this->coordinateFrame = -1;
    }

    Position::~Position()
    {
    }

    double & Position::operator[](int index)
    {
        assert(index >= 0);
        assert(index <= 2);

        return this->privateData[index];
    }

    const double & Position::operator[](int index) const
    {
        assert(index >= 0);
        assert(index <= 2);

        return this->privateData[index];
    }

    double & Position::operator()(int index)
    {
        assert(index >= 0);
        assert(index <= 2);

        return this->privateData[index];
    }

    const double & Position::operator()(int index) const
    {
        assert(index >= 0);
        assert(index <= 2);

        return this->privateData[index];
    }

    const double * Position::data() const
    {
        return this->privateData;
    }

    void Position::setCoordinateFrame(int _coordinateFrame)
    {
        this->coordinateFrame = _coordinateFrame;
    }

    int Position::getCoordinateFrame() const
    {
        return this->coordinateFrame;
    }

    void Position::setPoint(int _point)
    {
        this->point = _point;
    }

    int Position::getPoint() const
    {
        return this->point;
    }

    void Position::setReferencePoint(int _referencePoint)
    {
        this->referencePoint = _referencePoint;
    }

    int Position::getReferencePoint() const
    {
        this->referencePoint;
    }

    const Position& Position::changePointCoordinates(const Position& newPosition)
    {
        this->privateData[0] += newPosition(0);
        this->privateData[1] += newPosition(1);
        this->privateData[2] += newPosition(2);

        return *this;
    }

    bool Position::changePointSemantics(const Position& newPosition)
    {
        // check semantics
        if( !checkEqualOrUnknown(newPosition.coordinateFrame,this->coordinateFrame) )
        {
            fprintf(stderr,"[ERROR] Position::changePoint error: changePoint with newPosition expressed in a different coordinateFrames\n");
            assert(false);
            return false;
        }

        if( !checkEqualOrUnknown(newPosition.referencePoint,this->point) )
        {
            fprintf(stderr,"[ERROR] Position::changePoint error: newPosition has a refernce point different from the original point\n");
            assert(false);
            return false;
        }

        // set new semantics
        this->point = newPosition.point;
        return true;
    }

    const Position& Position::changePoint(const Position& newPosition)
    {
        if( changePointSemantics(newPosition) )
        {
            changePointCoordinates(newPosition);
        }

        return *this;
    }

    const Position& Position::changeRefPointCoordinates(const Position& newPosition)
    {
        this->privateData[0] += newPosition(0);
        this->privateData[1] += newPosition(1);
        this->privateData[2] += newPosition(2);

        return *this;
    }

    bool Position::changeRefPointSemantics(const Position& newPosition)
    {
        // check semantics
        if( !checkEqualOrUnknown(newPosition.coordinateFrame,this->coordinateFrame) )
        {
            fprintf(stderr,"[ERROR] Position::changeRefPoint error: changePoint with newPosition expressed in a different coordinateFrames\n");
            assert(false);
            return false;
        }

        if( !checkEqualOrUnknown(newPosition.point,this->referencePoint) )
        {
            fprintf(stderr,"[ERROR] Position::changeRefPoint error: newPosition has a refernce point different from the original point\n");
            assert(false);
            return false;
        }

        // set new semantics
        this->referencePoint = newPosition.referencePoint;
        return true;
    }

    const Position& Position::changeRefPoint(const Position& newPosition)
    {
        if( changeRefPointSemantics(newPosition) )
        {
            changeRefPointCoordinates(newPosition);
        }

        return *this;
    }

    void Position::composeCoordinates(const Position& op1, const Position& op2, Position& result)
    {
        result(0) = op1(0) + op2(0);
        result(1) = op1(1) + op2(1);
        result(2) = op1(2) + op2(2);
    }

    bool Position::composeSemantics(const Position& op1, const Position& op2, Position& result)
    {
        // check semantics
        if( !checkEqualOrUnknown(op1.coordinateFrame,op2.coordinateFrame) )
        {
            fprintf(stderr,"[ERROR] Position::compose error: composing two position expressed in different coordinateFrames\n");
            assert(false);
            return false;
        }

        if( !checkEqualOrUnknown(op1.referencePoint,op2.point) )
        {
            fprintf(stderr,"[ERROR] Position::compose error: composing two position where the reference point of the first one is different from the point of the second\n");
            if( op1.point == op2.referencePoint )
            {
                fprintf(stderr,"[ERROR] Position(a|A,c|C) = compose(Position(b|B,c|C),Position(a|A,b|B)) is forbidded in iDynTree to avoid ambiguity on compose(Position(b|B,a|A),Position(a|A,b|B))\n");
                assert(false);
                return false;
            }
        }

        // set new semantics
        result.referencePoint = op2.referencePoint;
        result.point = op1.point;
        result.coordinateFrame = op1.coordinateFrame;

        return true;
    }

    Position Position::compose(const Position& op1, const Position& op2)
    {
        Position result;

        if( composeSemantics(op1,op2,result) )
        {
            composeCoordinates(op1,op2,result);
        }

        return result;
    }

    void Position::inverseCoordinates(const Position& op, Position & result)
    {
        result(0) = -op.privateData[0];
        result(1) = -op.privateData[1];
        result(2) = -op.privateData[2];
    }

    bool Position::inverseSemantics(const Position& op, Position& result)
    {
        result.coordinateFrame = op.coordinateFrame;
        result.point = op.referencePoint;
        result.referencePoint = op.point;

        return true;
    }

    Position Position::inverse(const Position& op)
    {
        Position result;

        if( inverseSemantics(op,result) )
        {
            inverseCoordinates(op,result);
        }

        return result;

    }

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

        ss << "Coordinates:"
           << " " << this->data()[0]
           << " " << this->data()[1]
           << " " << this->data()[2]
           << " Semantics:"
           << " point " << this->getPoint()
           << " referencePoint " << this->getReferencePoint()
           << " coordinateFrame " << this->getCoordinateFrame();

        return ss.str();
    }




}