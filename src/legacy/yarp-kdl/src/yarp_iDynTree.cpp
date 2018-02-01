/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "iCub/iDynTree/yarp_iDynTree.h"

#include <yarp/math/Math.h>
#include <cstring>

using namespace yarp::math;



bool YarptoiDynTree(const yarp::sig::Vector & yarpVector, iDynTree::Wrench & iDynTreeWrench)
{
    if( yarpVector.size() != 6 )
    {
        return false;
    }

    memcpy(iDynTreeWrench.getLinearVec3().data(),yarpVector.data(),3*sizeof(double));
    memcpy(iDynTreeWrench.getAngularVec3().data(),yarpVector.data()+3,3*sizeof(double));
    return true;
}


bool iDynTreetoYarp(const iDynTree::Wrench & iDynTreeWrench,yarp::sig::Vector & yarpVector)
{
    if( yarpVector.size() != 6 )
    {
        yarpVector.resize(6);
    }

    memcpy(yarpVector.data(),iDynTreeWrench.getLinearVec3().data(),3*sizeof(double));
    memcpy(yarpVector.data()+3,iDynTreeWrench.getAngularVec3().data(),3*sizeof(double));
    return true;
}

bool YarptoiDynTree(const yarp::sig::Vector& yarpVector, iDynTree::Position& iDynTreePosition)
{
    if( yarpVector.size() != 3 )
    {
        return false;
    }

    memcpy(iDynTreePosition.data(),yarpVector.data(),3*sizeof(double));
    return true;
}

bool iDynTreetoYarp(const iDynTree::Position& iDynTreePosition, yarp::sig::Vector& yarpVector)
{
    if( yarpVector.size() != 3 )
    {
        yarpVector.resize(3);
    }

    memcpy(yarpVector.data(),iDynTreePosition.data(),3*sizeof(double));
    return true;
}




