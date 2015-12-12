/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/Utils.h>

#include <sstream>

#include <cassert>
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

VectorDynSize::VectorDynSize(const VectorDynSize& vec): m_size(vec.size())
{
    if( this->m_size == 0 )
    {
        this->m_data = 0;
    }
    else
    {
        this->m_data = new double[this->m_size];
        std::memcpy(this->m_data,vec.data(),this->m_size*sizeof(double));
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

VectorDynSize& VectorDynSize::operator=(const VectorDynSize& vec)
{
    // if the size don't match, reallocate the data
    if( this->m_size != vec.size() )
    {
        // Get the new size
        this->m_size = vec.size();

        // Destroy the data only if there is anything
        if( this->m_data != 0 )
        {
             delete[] this->m_data;
             this->m_data = 0;
        }

        // Actually allocate the data only if the size is not 0
        if( this->m_size > 0 )
        {
            this->m_data = new double[this->m_size];
        }

    }

    // After reallocation, the size should match
    assert(this->m_size == vec.size());

    // Copy data if the size is not 0
    if( this->m_size > 0 )
    {
        std::memcpy(this->m_data,vec.data(),this->m_size*sizeof(double));
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

void VectorDynSize::fillBuffer(double* buf) const
{
    for(unsigned int i=0; i < this->size(); i++ )
    {
        buf[i] = this->m_data[i];
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