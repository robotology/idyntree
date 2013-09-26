/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
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

