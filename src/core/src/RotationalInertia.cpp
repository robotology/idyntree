// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/RotationalInertia.h>
#include <iDynTree/Utils.h>

#include <iostream>
#include <sstream>

#include <cassert>
#include <cstring>


namespace iDynTree
{

RotationalInertia::RotationalInertia()
{
}


RotationalInertia::RotationalInertia(const double* in_data,
                                           const unsigned int in_rows,
                                           const unsigned int in_cols):
                                           Matrix3x3(in_data,in_rows,in_cols)
{
}

RotationalInertia::RotationalInertia(const RotationalInertia& other): Matrix3x3(other)
{
}

RotationalInertia& RotationalInertia::operator=(const RotationalInertia& other)
{
    for(std::size_t row=0; row < 3; row++ )
    {
        for(std::size_t col=0; col < 3; col++ )
        {
            this->m_data[row*3+col] = other.m_data[row*3+col];
        }
    }
    return *this;
}

RotationalInertia RotationalInertia::Zero()
{
    RotationalInertia ret;
    ret.zero();
    return ret;
}


}
