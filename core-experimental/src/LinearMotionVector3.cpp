/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "LinearMotionVector3.h"

namespace iDynTree
{
    // constructors
    LinearMotionVector3::LinearMotionVector3()
    {
        // MotionVector3<LinearMotionVector3, LinearMotionAssociationsT>() will be implicitly called
    }
    
    LinearMotionVector3::LinearMotionVector3(const double* in_data, const unsigned int in_size): MotionVector3<LinearMotionVector3, LinearMotionAssociationsT>(in_data, in_size)
    {
    }
    
    LinearMotionVector3::LinearMotionVector3(const LinearMotionVector3 & other): MotionVector3<LinearMotionVector3, LinearMotionAssociationsT>(other)
    {
    }
    
    LinearMotionVector3::~LinearMotionVector3()
    {
    }
}