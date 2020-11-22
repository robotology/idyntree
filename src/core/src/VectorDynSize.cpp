/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/Utils.h>

#include <sstream>

#include <cassert>
#include <cstring>

namespace iDynTree
{

VectorDynSize::VectorDynSize(): m_data(0), m_size(0), m_capacity(0)
{

}

VectorDynSize::VectorDynSize(std::size_t _size): m_size(_size), m_capacity(_size)
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
                             const std::size_t in_size): m_size(in_size), m_capacity(in_size)
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

VectorDynSize::VectorDynSize(const VectorDynSize& vec): m_size(vec.size()), m_capacity(vec.capacity())
{
    if( this->m_size == 0 )
    {
        this->m_data = 0;
    }
    else
    {
        this->m_data = new double[this->m_capacity];
        std::memcpy(this->m_data,vec.data(),this->m_capacity*sizeof(double));
    }
}

VectorDynSize::~VectorDynSize()
{
    if( this->m_capacity > 0 )
    {
        delete[] this->m_data;
        this->m_data = 0;
    }
}
#if !defined(SWIG_VERSION) || SWIG_VERSION >= 0x030000
VectorDynSize::VectorDynSize(Span<const double> vec)
    : VectorDynSize(vec.data(), vec.size())
{}
#endif

VectorDynSize& VectorDynSize::operator=(const VectorDynSize& vec)
{
    // if the size don't match, reallocate the data
    if( this->m_size != vec.size() )
    {
        resize(vec.size());
    }

    // After reallocation, the size should match
    assert(this->m_size == vec.size());

    // Copy data if the size is not 0
    if( this->m_size > 0 )
    {
        std::memcpy(this->m_data,vec.data(),this->m_size*sizeof(double));
    }

    return *this;
}

#if !defined(SWIG_VERSION) || SWIG_VERSION >= 0x030000
VectorDynSize &VectorDynSize::operator=(Span<const double> vec)
{
    // if the size don't match, reallocate the data
    if( this->m_size != vec.size() )
    {
        resize(static_cast<std::size_t>(vec.size()));
    }

    // After reallocation, the size should match
    assert(this->m_size == vec.size());

    // Copy data if the size is not 0
    if( this->m_size > 0 )
    {
        std::memcpy(this->m_data, vec.data(), this->m_size*sizeof(double));
    }

    return *this;
}
#endif

void VectorDynSize::zero()
{
    for(std::size_t i=0; i < this->size(); i++ )
    {
        this->m_data[i] = 0.0;
    }
}


std::size_t VectorDynSize::size() const
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

double& VectorDynSize::operator()(std::size_t index)
{
    assert(index < this->size());
    return this->m_data[index];
}

double VectorDynSize::operator()(std::size_t index) const
{
    return this->m_data[index];
}

double& VectorDynSize::operator[](std::size_t index)
{
    assert(index < this->size());
    return this->m_data[index];
}

double VectorDynSize::operator[](std::size_t index) const
{
    assert(index < this->size());
    return this->m_data[index];
}

double VectorDynSize::getVal(const std::size_t index) const
{
    if( index >= this->size() )
    {
        reportError("VectorDynSize","getVal","index out of bounds");
        return 0.0;
    }

    return this->m_data[index];
}

bool VectorDynSize::setVal(const std::size_t index, const double new_el)
{
    if( index >= this->size() )
    {
        reportError("VectorDynSize","getVal","index out of bounds");
        return false;
    }

    this->m_data[index] = new_el;

    return true;
}

const double* VectorDynSize::begin() const noexcept
{
    return this->m_data;
}

const double* VectorDynSize::end() const noexcept
{
    return this->m_data + this->m_size;
}

const double* VectorDynSize::cbegin() const noexcept
{
    return this->m_data;
}

const double* VectorDynSize::cend() const noexcept
{
    return this->m_data + this->m_size;
}

double* VectorDynSize::begin() noexcept
{
    return this->m_data;
}

double* VectorDynSize::end() noexcept
{
    return this->m_data + this->m_size;
}

void VectorDynSize::changeCapacityAndCopyData(const std::size_t _newCapacity)
{
    //Corner case: zero capacity
    if (_newCapacity == 0) {
        delete [] this->m_data;
        m_data = 0;
        this->m_size = 0;
        this->m_capacity = 0;
        return;
    }

    double *localBuf = new double[_newCapacity];

    if (this->m_data && this->m_size <= _newCapacity) {
        memcpy(localBuf, this->m_data, this->m_size * sizeof(double));
    }
    if (this->m_data) {
        delete [] this->m_data;
        this->m_data = 0;
    }

    this->m_capacity = _newCapacity;
    this->m_data = localBuf;
}

void VectorDynSize::reserve(const std::size_t _newCapacity)
{
    assert(this->m_size <= this->m_capacity);

    if( _newCapacity <= this->m_capacity )
    {
        return;
    }

    this->changeCapacityAndCopyData(_newCapacity);
}

void VectorDynSize::resize(const std::size_t _newSize)
{
    //Possible cases:
    // 1) Reduce size -> don't touch capacity. Lose elements > _newSize
    // 2) Increase size
    //    a) capacity already at least _newSize
    //    b) capacity < _newSize
    reserve(_newSize);
    this->m_size = _newSize;
}

size_t VectorDynSize::capacity() const
{
    return this->m_capacity;
}

void VectorDynSize::shrink_to_fit()
{
    if( this->m_size == this->m_capacity )
    {
        return;
    }

    this->changeCapacityAndCopyData(this->m_size);
}



void VectorDynSize::fillBuffer(double* buf) const
{
    for(std::size_t i=0; i < this->size(); i++ )
    {
        buf[i] = this->m_data[i];
    }
}


std::string VectorDynSize::toString() const
{
    std::stringstream ss;

    for(std::size_t i=0; i < this->size(); i++ )
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
