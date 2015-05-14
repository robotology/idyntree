/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "Vector3.h"
#include "Utils.h"
#include <sstream>
#include <cstring>

namespace iDynTree
{

const unsigned int Vector3Size = 3;

Vector3::Vector3()
{
    this->zero();
}

Vector3::Vector3(const double* in_data,
                 const unsigned int in_size)
{
    if( in_size != Vector3Size )
    {
        reportError("Vector3","constructor","input vector does not have 6 elements");
        this->zero();
    }
    else
    {
        memcpy(this->m_data,in_data,sizeof(double)*Vector3Size);
    }
}

Vector3::~Vector3()
{
}

void Vector3::zero()
{
    for(int i=0; i < this->size(); i++ )
    {
        this->m_data[i] = 0.0;
    }
}


unsigned int Vector3::size() const
{
    return Vector3Size;
}


double* Vector3::data()
{
    return this->m_data;
}

const double* Vector3::data() const
{
    return this->m_data;
}

double& Vector3::operator()(unsigned int index)
{
    return this->m_data[index];
}

double Vector3::operator()(unsigned int index) const
{
    return this->m_data[index];
}

double Vector3::getVal(const unsigned int index) const
{
    if( index > this->size() )
    {
        reportError("Vector3","getVal","index out of bounds");
        return 0.0;
    }

    return this->m_data[index];
}

bool Vector3::setVal(const unsigned int index, const double new_el)
{
    if( index > this->size() )
    {
        reportError("Vector3","getVal","index out of bounds");
        return false;
    }

    this->m_data[index] = new_el;

    return true;
}

std::string Vector3::toString() const
{
    std::stringstream ss;

    for(int i=0; i < this->size(); i++ )
    {
        ss << this->m_data[i] << " ";
    }

    return ss.str();
}

std::string Vector3::reservedToString() const
{
    return this->toString();
}


}