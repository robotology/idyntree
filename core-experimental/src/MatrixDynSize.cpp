/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "MatrixDynSize.h"
#include "Utils.h"
#include <cassert>
#include <cstdio>
#include <sstream>
#include <cstring>

namespace iDynTree
{

MatrixDynSize::MatrixDynSize(): m_data(0), m_rows(0), m_cols(0)
{

}

MatrixDynSize::MatrixDynSize(unsigned int _rows,
                             unsigned int _cols): m_rows(_rows),
                                                  m_cols(_cols)
{
    if( this->m_rows*this->m_cols == 0 )
    {
        this->m_data = 0;
    }
    else
    {
        this->m_data = new double[this->m_rows*this->m_cols];
    }

    zero();
}


MatrixDynSize::MatrixDynSize(const double* in_data,
                             const unsigned int in_rows,
                             const unsigned int in_cols): m_rows(in_rows),
                                                          m_cols(in_cols)
{
    if( this->m_rows*this->m_cols == 0 )
    {
        this->m_data = 0;
    }
    else
    {
        this->m_data = new double[this->m_rows*this->m_cols];
        std::memcpy(this->m_data,in_data,in_rows*in_cols*sizeof(double));
    }
}

MatrixDynSize::~MatrixDynSize()
{
    if( this->m_rows*this->m_cols > 0 )
    {
        delete[] this->m_data;
        this->m_data = 0;
    }
}

void MatrixDynSize::zero()
{
    for(int row=0; row < this->rows(); row++ )
    {
        for(int col=0; col < this->cols(); col++ )
        {
            this->m_data[rawIndex(row,col)] = 0.0;
        }
    }
}

unsigned int MatrixDynSize::rows() const
{
    return this->m_rows;
}

unsigned int MatrixDynSize::cols() const
{
    return this->m_cols;
}

double* MatrixDynSize::data()
{
    return this->m_data;
}

const double* MatrixDynSize::data() const
{
    return this->m_data;
}

double& MatrixDynSize::operator()(const unsigned int row, const unsigned int col)
{
    return this->m_data[rawIndex(row,col)];
}

double MatrixDynSize::operator()(const unsigned int row, const unsigned int col) const
{
    return this->m_data[rawIndex(row,col)];
}

double MatrixDynSize::getVal(const unsigned int row, const unsigned int col) const
{
    if( row > this->rows() ||
        col  > this->cols() )
    {
        reportError("MatrixDynSize","getVal","indeces out of bounds");
        return 0.0;
    }

    return this->m_data[rawIndex(row,col)];
}

bool MatrixDynSize::setVal(const unsigned int row, const unsigned int col, const double new_el)
{
    if( row > this->rows() ||
        col   > this->cols() )
    {
        reportError("MatrixDynSize","setVal","indeces out of bounds");
        return false;
    }

    this->m_data[rawIndex(row,col)] = new_el;
    return true;
}

void MatrixDynSize::resize(const unsigned int _newRows, const unsigned int _newCols)
{
    if( (_newRows == this->rows()) &&
        (_newCols == this->cols()) )
    {
        return;
    }

    if( this->m_data )
    {
        delete[] this->m_data;
    }

    this->m_rows = _newRows;
    this->m_cols = _newCols;

    if( this->m_rows*this->m_cols == 0 )
    {
        this->m_data = 0;
    }
    else
    {
        this->m_data = new double[this->m_cols*this->m_rows];
        zero();
    }
}

const unsigned int MatrixDynSize::rawIndex(int row, int col) const
{
    return (this->m_cols*row + col);
}


std::string MatrixDynSize::toString() const
{
    std::stringstream ss;

    for(int row=0; row < this->rows(); row++ )
    {
        for(int col=0; col < this->cols(); col++ )
        {
            ss << this->m_data[this->rawIndex(row,col)] << " ";
        }
        ss << std::endl;
    }

    return ss.str();
}




}