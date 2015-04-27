/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "PositionRaw.h"
#include "Utils.h"
#include <cassert>
#include <cstdio>
#include <sstream>

namespace iDynTree
{
    PositionRaw::PositionRaw()
    {
        this->privateData[0] = this->privateData[1] = this->privateData[2] = 0.0;
    }


    PositionRaw::PositionRaw(double x, double y, double z)
    {
        this->privateData[0] = x;
        this->privateData[1] = y;
        this->privateData[2] = z;
    }


    PositionRaw::PositionRaw(const PositionRaw& other)
    {
        this->privateData[0] = other.privateData[0];
        this->privateData[1] = other.privateData[1];
        this->privateData[2] = other.privateData[2];

    }

    PositionRaw::~PositionRaw()
    {

    }

    double & PositionRaw::operator()(int index)
    {
        assert(index >= 0);
        assert(index <= 2);

        return this->privateData[index];
    }

    const double & PositionRaw::operator()(int index) const
    {
        assert(index >= 0);
        assert(index <= 2);

        return this->privateData[index];
    }

    const double * PositionRaw::data() const
    {
        return this->privateData;
    }

    double * PositionRaw::data()
    {
        return this->privateData;
    }

    const PositionRaw& PositionRaw::changePoint(const PositionRaw& newPoint)
    {
        this->privateData[0] += newPoint(0);
        this->privateData[1] += newPoint(1);
        this->privateData[2] += newPoint(2);

        return *this;
    }

    const PositionRaw& PositionRaw::changeRefPoint(const PositionRaw& newPosition)
    {
        this->privateData[0] += newPosition(0);
        this->privateData[1] += newPosition(1);
        this->privateData[2] += newPosition(2);

        return *this;
    }

    PositionRaw PositionRaw::compose(const PositionRaw& op1, const PositionRaw& op2)
    {
        PositionRaw result;
        result(0) = op1(0) + op2(0);
        result(1) = op1(1) + op2(1);
        result(2) = op1(2) + op2(2);
        return result;
    }

    PositionRaw PositionRaw::inverse(const PositionRaw& op)
    {
        PositionRaw result;
        result(0) = -op.privateData[0];
        result(1) = -op.privateData[1];
        result(2) = -op.privateData[2];
        return result;
    }


    // overloaded operators
    PositionRaw PositionRaw::operator+(const PositionRaw& other) const
    {
        return compose(*this,other);
    }

    PositionRaw PositionRaw::operator-() const
    {
        return inverse(*this);
    }

    PositionRaw PositionRaw::operator-(const PositionRaw& other) const
    {
        return compose(*this,inverse(other));
    }

    std::string PositionRaw::toString() const
    {
        std::stringstream ss;

        ss << " x " << this->privateData[0]
           << " y " << this->privateData[1]
           << " z " << this->privateData[2];

        return ss.str();
    }




}