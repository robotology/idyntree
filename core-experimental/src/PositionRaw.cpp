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
    this->zero();
}


PositionRaw::PositionRaw(double x, double y, double z)
{
    this->m_data[0] = x;
    this->m_data[1] = y;
    this->m_data[2] = z;
}


PositionRaw::PositionRaw(const PositionRaw& other)
{
    this->m_data[0] = other.m_data[0];
    this->m_data[1] = other.m_data[1];
    this->m_data[2] = other.m_data[2];

}

PositionRaw::~PositionRaw()
{

}

double & PositionRaw::operator()(const unsigned int index)
{
    return this->m_data[index];
}

double PositionRaw::operator()(const unsigned int index) const
{
    return this->m_data[index];
}

double PositionRaw::getVal(const unsigned int index) const
{
    if( index > this->size() )
    {
        reportError("PositionRaw","getVal","index out of bounds");
        return 0.0;
    }

    return this->m_data[index];
}

bool PositionRaw::setVal(const unsigned int index, const double new_el)
{
    if( index > this->size() )
    {
        reportError("PositionRaw","getVal","index out of bounds");
        return false;
    }

    this->m_data[index] = new_el;

    return true;
}

void PositionRaw::zero()
{
    this->m_data[0] = this->m_data[1] = this->m_data[2] = 0.0;
}

unsigned int PositionRaw::size() const
{
    return 3;
}

const double * PositionRaw::data() const
{
    return this->m_data;
}

double * PositionRaw::data()
{
    return this->m_data;
}

const PositionRaw& PositionRaw::changePoint(const PositionRaw& newPoint)
{
    this->m_data[0] += newPoint(0);
    this->m_data[1] += newPoint(1);
    this->m_data[2] += newPoint(2);

    return *this;
}

const PositionRaw& PositionRaw::changeRefPoint(const PositionRaw& newPosition)
{
    this->m_data[0] += newPosition(0);
    this->m_data[1] += newPosition(1);
    this->m_data[2] += newPosition(2);

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
    result(0) = -op.m_data[0];
    result(1) = -op.m_data[1];
    result(2) = -op.m_data[2];
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

    ss << " x " << this->m_data[0]
       << " y " << this->m_data[1]
       << " z " << this->m_data[2];

    return ss.str();
}

std::string PositionRaw::reservedToString() const
{
    return this->toString();
}




}