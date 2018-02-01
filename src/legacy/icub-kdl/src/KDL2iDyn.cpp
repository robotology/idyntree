/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include "KDL2iDyn.h"

bool to_iDyn(const KDL::Vector & kdlVector,yarp::sig::Vector & idynVector)
{
    idynVector = yarp::sig::Vector(3);
    idynVector[0] = kdlVector[0];
    idynVector[1] = kdlVector[1];
    idynVector[2] = kdlVector[2];
    return true;
}

bool kdlJntArray2idynVector(const KDL::JntArray & kdlJntArray,yarp::sig::Vector & idynVector)
{
    int nj = kdlJntArray.rows();
    idynVector = yarp::sig::Vector(nj);
    for(int i=0;i<nj;i++) {
        idynVector[i] = kdlJntArray(i);
    }
    return true;
}

