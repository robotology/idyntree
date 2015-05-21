/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "VectorDynSize.h"
#include "Utils.h"
#include <sstream>
#include <cstring>

namespace iDynTree
{

VectorDynSize::VectorDynSize(): m_data(0), m_size(0)
{

}

VectorDynSize::VectorDynSize(unsigned int _size): m_size(_size)
{
    if( this->m_size == 0 )
    {
        this->m_data = 0;
    }
    else
    {
        this->m_data = new double[this->m_size];
    }

    zero();
}


VectorDynSize::VectorDynSize(const double* in_data,
                             const unsigned int in_size): m_size(in_size)
{
    if( this->m_size == 0 )
    {
        this->m_data = 0;
    }
    else
    {
        this->m_data = new double[this->m_size];
        std::memcpy(this->m_data,in_data,in_size*sizeof(double));
    }
}

VectorDynSize::~VectorDynSize()
{
    if( this->m_size > 0 )
    {
        delete[] this->m_data;
        this->m_data = 0;
    }
}

void VectorDynSize::zero()
{
    for(unsigned int i=0; i < this->size(); i++ )
    {
        this->m_data[i] = 0.0;
    }
}


unsigned int VectorDynSize::size() const
{
    return this->m_size;
}


double* VectorDynSize::data()
{
    return this->m_data;
}

const double* VectorDynSize::data() const
{
    return this->m_data;
}

double& VectorDynSize::operator()(unsigned int index)
{
    return this->m_data[index];
}

double VectorDynSize::operator()(unsigned int index) const
{
    return this->m_data[index];
}

double VectorDynSize::getVal(const unsigned int index) const
{
    if( index > this->size() )
    {
        reportError("VectorDynSize","getVal","index out of bounds");
        return 0.0;
    }

    return this->m_data[index];
}

bool VectorDynSize::setVal(const unsigned int index, const double new_el)
{
    if( index > this->size() )
    {
        reportError("VectorDynSize","getVal","index out of bounds");
        return false;
    }

    this->m_data[index] = new_el;

    return true;
}

void VectorDynSize::resize(const unsigned int _newSize)
{
    if( _newSize == this->size() )
    {
        return;
    }

    if( this->m_data )
    {
        delete[] this->m_data;
    }

    this->m_size = _newSize;

    if( this->m_size == 0 )
    {
        this->m_data = 0;
    }
    else
    {
        this->m_data = new double[this->m_size];
        zero();
    }
}



std::string VectorDynSize::toString() const
{
    std::stringstream ss;

    for(unsigned int i=0; i < this->size(); i++ )
    {
        ss << this->m_data[i] << " ";
    }

    return ss.str();
}

std::string VectorDynSize::reservedToString() const
{
    return this->toString();
}





}