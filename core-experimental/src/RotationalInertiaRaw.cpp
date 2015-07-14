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

RotationalInertiaRaw::RotationalInertiaRaw()
{
    this->zero();
}


RotationalInertiaRaw::RotationalInertiaRaw(const double* in_data,
                                           const unsigned int in_rows,
                                           const unsigned int in_cols):
                                           Matrix3x3(in_data,in_rows,in_cols)
{
}

RotationalInertiaRaw::RotationalInertiaRaw(const RotationalInertiaRaw& other): Matrix3x3(other)
{
}

RotationalInertiaRaw::~RotationalInertiaRaw()
{

}


}