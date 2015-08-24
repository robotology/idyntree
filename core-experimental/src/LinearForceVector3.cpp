/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "LinearForceVector3.h"

namespace iDynTree
{
    /**
     * LinearForceVector3Semantics
     */
    
    // constructors
    LinearForceVector3Semantics::LinearForceVector3Semantics()
    {
    }

    LinearForceVector3Semantics::LinearForceVector3Semantics(int _body, int _refBody, int _coordinateFrame):
    ForceVector3Semantics<LinearForceVector3Semantics>(_body, _refBody, _coordinateFrame)
    {
    }

    LinearForceVector3Semantics::LinearForceVector3Semantics(const LinearForceVector3Semantics & other):
    ForceVector3Semantics<LinearForceVector3Semantics>(other)
    {
    }

    LinearForceVector3Semantics::~LinearForceVector3Semantics()
    {
    }


    /**
     * LinearForceVector3
     */
    
    // constructors
    LinearForceVector3::LinearForceVector3()
    {
    }
    
    LinearForceVector3::LinearForceVector3(const double* in_data, const unsigned int in_size):
    ForceVector3<LinearForceVector3, LinearForceAssociationsT, LinearForceVector3Semantics>(in_data, in_size)
    {
    }
    
    LinearForceVector3::LinearForceVector3(const LinearForceVector3 & other):
    ForceVector3<LinearForceVector3, LinearForceAssociationsT, LinearForceVector3Semantics>(other)
    {
    }
    
    LinearForceVector3::~LinearForceVector3()
    {
    }
}