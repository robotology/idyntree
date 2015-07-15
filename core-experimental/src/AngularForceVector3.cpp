/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "AngularForceVector3.h"

namespace iDynTree
{
    AngularForceVector3::AngularForceVector3()
    {
        // ForceVector3<AngularForceVector3>() will be implicitly called
    }
    
    
    AngularForceVector3::AngularForceVector3(const double* in_data, const unsigned int in_size): ForceVector3<AngularForceVector3>(in_data, in_size)
    {
    }
    
    AngularForceVector3::AngularForceVector3(const AngularForceVector3& other): ForceVector3<AngularForceVector3>(other)
    {
    }
    
    AngularForceVector3::~AngularForceVector3()
    {
    }

}