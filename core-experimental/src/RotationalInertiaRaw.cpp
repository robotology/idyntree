/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "RotationalInertiaRaw.h"
#include "Utils.h"
#include <cassert>
#include <iostream>
#include <sstream>
#include <cstring>


namespace iDynTree
{

const unsigned int RotationalInertiaRawRows = 3;
const unsigned int RotationalInertiaRawCols = 3;
const unsigned int RotationalInertiaRawTotSize = 9;

RotationalInertiaRaw::RotationalInertiaRaw()
{
    this->zero();
}


RotationalInertiaRaw::RotationalInertiaRaw(const double* in_data,
                                           const unsigned int in_rows,
                                           const unsigned int in_cols)
{
    if( in_rows != RotationalInertiaRawRows ||
        in_cols != RotationalInertiaRawCols )
    {
        reportError("RotationalInertiaRaw","constructor","input matrix does not have size 3x3");
        this->zero();
    }
    else
    {
        memcpy(this->m_data,in_data,sizeof(double)*RotationalInertiaRawTotSize);
    }
}

RotationalInertiaRaw::RotationalInertiaRaw(const RotationalInertiaRaw& other)
{
    memcpy(this->m_data,other.m_data,RotationalInertiaRawTotSize*sizeof(double));
}

RotationalInertiaRaw::~RotationalInertiaRaw()
{

}

unsigned int RotationalInertiaRaw::rows() const
{
    return RotationalInertiaRawRows;
}

unsigned int RotationalInertiaRaw::cols() const
{
    return RotationalInertiaRawCols;
}

double& RotationalInertiaRaw::operator()(const unsigned int row, const unsigned int col)
{
    return this->m_data[RotationalInertiaRawCols*row + col];
}

double RotationalInertiaRaw::operator()(const unsigned int row, const unsigned int col) const
{
    return this->m_data[RotationalInertiaRawCols*row + col];
}

bool RotationalInertiaRaw::setVal(const unsigned int row, const unsigned int col, const double new_el)
{
    if( row >= this->rows() ||
        col >= this->cols() )
    {
        reportError("RotationalInertiaRaw","setVal","indeces out of bounds");
        return false;
    }

    this->operator()(row,col) = new_el;

    return true;
}

double RotationalInertiaRaw::getVal(const unsigned int row, const unsigned int col) const
{
    if( row >= this->rows() ||
        col >= this->cols() )
    {
        reportError("RotationalInertiaRaw","getVal","indeces out of bounds");
        return 0.0;
    }

    return this->operator()(row,col);
}

void RotationalInertiaRaw::zero()
{
    for(unsigned int r=0; r < RotationalInertiaRawRows; r++ )
    {
        for(unsigned int c=0; c < RotationalInertiaRawCols; c++ )
        {
            this->operator()(r,c) = 0.0;
        }
    }
    return;
}

const double * RotationalInertiaRaw::data() const
{
    return this->m_data;
}

double * RotationalInertiaRaw::data()
{
    return this->m_data;
}



}