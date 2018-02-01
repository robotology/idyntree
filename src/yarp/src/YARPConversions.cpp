/*
 * Copyright (C) 2013 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/yarp/YARPConversions.h>

#include <iDynTree/Core/Direction.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/VectorDynSize.h>

#include <yarp/math/Math.h>
#include <cstring>

using namespace yarp::math;

namespace iDynTree
{

bool toiDynTree(const yarp::sig::Vector & yarpVector, iDynTree::Wrench & iDynTreeWrench)
{
    if( yarpVector.size() != 6 )
    {
        return false;
    }

    memcpy(iDynTreeWrench.getLinearVec3().data(),yarpVector.data(),3*sizeof(double));
    memcpy(iDynTreeWrench.getAngularVec3().data(),yarpVector.data()+3,3*sizeof(double));
    return true;
}


bool toYarp(const iDynTree::Wrench & iDynTreeWrench,yarp::sig::Vector & yarpVector)
{
    if( yarpVector.size() != 6 )
    {
        yarpVector.resize(6);
    }

    memcpy(yarpVector.data(),iDynTreeWrench.getLinearVec3().data(),3*sizeof(double));
    memcpy(yarpVector.data()+3,iDynTreeWrench.getAngularVec3().data(),3*sizeof(double));
    return true;
}

bool toiDynTree(const yarp::sig::Vector& yarpVector, iDynTree::Position& iDynTreePosition)
{
    if( yarpVector.size() != 3 )
    {
        return false;
    }

    memcpy(iDynTreePosition.data(),yarpVector.data(),3*sizeof(double));
    return true;
}

bool toiDynTree(const yarp::sig::Vector& yarpVector, iDynTree::Vector3& iDynTreeVector3)
{
    if( yarpVector.size() != 3 )
    {
        return false;
    }

    memcpy(iDynTreeVector3.data(),yarpVector.data(),3*sizeof(double));
    return true;
}

bool toYarp(const iDynTree::Position& iDynTreePosition, yarp::sig::Vector& yarpVector)
{
    if( yarpVector.size() != 3 )
    {
        yarpVector.resize(3);
    }

    memcpy(yarpVector.data(),iDynTreePosition.data(),3*sizeof(double));
    return true;
}

bool toiDynTree(const yarp::sig::Vector& yarpVector, Direction& direction)
{
    if( yarpVector.size() != 3 )
    {
        return false;
    }

    memcpy(direction.data(),yarpVector.data(),3*sizeof(double));

    // normalize
    direction.Normalize();

    return true;
}

bool toYarp(const Vector3& iDynTreeVector3, yarp::sig::Vector& yarpVector)
{
    if( yarpVector.size() != 3 )
    {
        yarpVector.resize(3);
    }

    memcpy(yarpVector.data(),iDynTreeVector3.data(),3*sizeof(double));
    return true;
}

bool toiDynTree(const yarp::sig::Vector& yarpVector, VectorDynSize& iDynTreeVector)
{
    iDynTreeVector.resize(yarpVector.size());
    memcpy(iDynTreeVector.data(),yarpVector.data(),yarpVector.size()*sizeof(double));
    return true;
}

bool toiDynTree(const yarp::sig::Matrix& yarpHomogeneousMatrix,
                iDynTree::Transform& iDynTreeTransform)
{
    if( yarpHomogeneousMatrix.rows() != 4 ||
        yarpHomogeneousMatrix.cols() != 4 )
    {
        reportError("","toiDynTree","Input yarp homegeneous matrix is not 4x4");
        return false;
    }

    Rotation rot;
    for(int r=0; r<3; r++)
    {
        for( int c=0; c<3; c++)
        {
            rot(r,c) = yarpHomogeneousMatrix(r,c);
        }
    }

    Position pos;
    for(int i=0; i<3; i++)
    {
        pos(i) = yarpHomogeneousMatrix(i,3);
    }

    iDynTreeTransform.setPosition(pos);
    iDynTreeTransform.setRotation(rot);

    return true;
}

bool toYarp(const iDynTree::Transform& iDynTreeTransform,
            yarp::sig::Matrix& yarpHomogeneousMatrix)
{
    iDynTree::Matrix4x4 homTrans = iDynTreeTransform.asHomogeneousTransform();

    toYarp(homTrans,yarpHomogeneousMatrix);

    return true;
}






}




