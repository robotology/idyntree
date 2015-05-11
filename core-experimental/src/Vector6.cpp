/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "Vector6.h"
#include "Utils.h"
#include <sstream>
#include <cstring>

namespace iDynTree
{

const unsigned int Vector6Size = 6;

Vector6::Vector6()
{
    this->zero();
}

Vector6::Vector6(const double* in_data,
                 const unsigned int in_size)
{
    if( in_size != Vector6Size )
    {
        reportError("Vector6","constructor","input vector does not have 6 elements");
        this->zero();
    }
    else
    {
        memcpy(this->m_data,in_data,sizeof(double)*Vector6Size);
    }
}

Vector6::~Vector6()
{
}

void Vector6::zero()
{
    for(int i=0; i < this->size(); i++ )
    {
        this->m_data[i] = 0.0;
    }
}


unsigned int Vector6::size() const
{
    return Vector6Size;
}


double* Vector6::data()
{
    return this->m_data;
}

const double* Vector6::data() const
{
    return this->m_data;
}

double& Vector6::operator()(unsigned int index)
{
    return this->m_data[index];
}

double Vector6::operator()(unsigned int index) const
{
    return this->m_data[index];
}

double Vector6::getVal(const unsigned int index) const
{
    if( index > this->size() )
    {
        reportError("Vector6","getVal","index out of bounds");
        return 0.0;
    }

    return this->m_data[index];
}

bool Vector6::setVal(const unsigned int index, const double new_el)
{
    if( index > this->size() )
    {
        reportError("Vector6","getVal","index out of bounds");
        return false;
    }

    this->m_data[index] = new_el;

    return true;
}

std::string Vector6::toString() const
{
    std::stringstream ss;

    for(int i=0; i < this->size(); i++ )
    {
        ss << this->m_data[i] << " ";
    }

    return ss.str();
}


}