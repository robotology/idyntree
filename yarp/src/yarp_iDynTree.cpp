/*
 * Copyright (C) 2013 IIT - Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 *
 */

#include "iCub/iDynTree/yarp_iDynTree.h"

#include <yarp/math/Math.h>
#include <cstring>

using namespace yarp::math;



bool YarptoiDynTree(const yarp::sig::Vector & yarpVector, iDynTree::Wrench & iDynTreeWrench)
{
    if( yarpVector.size() != 6 ) return false;
    memcpy(iDynTreeWrench.data(),yarpVector.data(),6*sizeof(double));
    return true;
}


bool iDynTreetoYarp(const iDynTree::Wrench & iDynTreeWrench,yarp::sig::Vector & yarpVector)
{
    if( yarpVector.size() != 6 ) { yarpVector.resize(6); }
    memcpy(yarpVector.data(),iDynTreeWrench.data(),6*sizeof(double));
    return true;
}



