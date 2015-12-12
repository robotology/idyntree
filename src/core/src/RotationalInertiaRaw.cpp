/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/RotationalInertiaRaw.h>
#include <iDynTree/Core/Utils.h>

#include <iostream>
#include <sstream>

#include <cassert>
#include <cstring>


namespace iDynTree
{

RotationalInertiaRaw::RotationalInertiaRaw()
{
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

RotationalInertiaRaw RotationalInertiaRaw::Zero()
{
    RotationalInertiaRaw ret;
    ret.zero();
    return ret;
}


}