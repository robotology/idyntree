/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "AngularMotionVector3.h"

namespace iDynTree
{
    // constructors
    AngularMotionVector3::AngularMotionVector3()
    {
        // MotionVector3<AngularMotionVector3, AngularMotionAssociationsT>() will be implicitly called
    }
    
    
    AngularMotionVector3::AngularMotionVector3(const double* in_data, const unsigned int in_size): MotionVector3<AngularMotionVector3, AngularMotionAssociationsT>(in_data, in_size)
    {
    }
    
    AngularMotionVector3::AngularMotionVector3(const AngularMotionVector3& other): MotionVector3<AngularMotionVector3, AngularMotionAssociationsT>(other)
    {
    }
    
    AngularMotionVector3::~AngularMotionVector3()
    {
    }

}