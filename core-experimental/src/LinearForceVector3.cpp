/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "LinearForceVector3.h"

namespace iDynTree
{
    // constructors
    LinearForceVector3::LinearForceVector3()
    {
        // ForceVector3<LinearForceVector3, LinearForceAssociationsT>() will be implicitly called
    }
    
    LinearForceVector3::LinearForceVector3(const double* in_data, const unsigned int in_size): ForceVector3<LinearForceVector3, LinearForceAssociationsT>(in_data, in_size)
    {
    }
    
    LinearForceVector3::LinearForceVector3(const LinearForceVector3 & other): ForceVector3<LinearForceVector3, LinearForceAssociationsT>(other)
    {
    }
    
    LinearForceVector3::~LinearForceVector3()
    {
    }
}