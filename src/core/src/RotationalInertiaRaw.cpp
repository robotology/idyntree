/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
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
