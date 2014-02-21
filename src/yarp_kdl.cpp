/*
 * Copyright (C) 2013 IIT - Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the GNU LGPL v2.0 (or any later version)
 *
 */
 
#include "iCub/iDynTree/yarp_kdl.h"

#include <yarp/math/Math.h>
#include <cstring>

using namespace yarp::math;

bool YarptoKDL(const yarp::sig::Matrix & idynInertia,KDL::RotationalInertia & kdlRotationalInertia)
{
     if(idynInertia.cols() != 3 || idynInertia.rows() != 3 ) return false;
     if(idynInertia(0,1) != idynInertia(1,0) || idynInertia(0,2) != idynInertia(2,0) || idynInertia(1,2) != idynInertia(2,1)) return false;
     kdlRotationalInertia = KDL::RotationalInertia(idynInertia(0,0),idynInertia(1,1),idynInertia(2,2),idynInertia(0,1),idynInertia(0,2),idynInertia(1,2));
     return true;
}

bool YarptoKDL(const yarp::sig::Matrix & idynMatrix, KDL::Frame & kdlFrame)
{
    if( idynMatrix.cols() != 4 || idynMatrix.rows() != 4 ) return false;
    KDL::Rotation kdlRotation;
    KDL::Vector kdlVector;
    YarptoKDL(idynMatrix.submatrix(0,2,0,2),kdlRotation);
    YarptoKDL(idynMatrix.subcol(0,3,3),kdlVector);
    kdlFrame = KDL::Frame(kdlRotation,kdlVector);
    return true;
}

bool YarptoKDL(const yarp::sig::Matrix & idynMatrix, KDL::Rotation & kdlRotation)
{
    if(idynMatrix.cols() != 3 || idynMatrix.rows() != 3) return false;
    kdlRotation = KDL::Rotation(idynMatrix(0,0),idynMatrix(0,1),idynMatrix(0,2),
                                idynMatrix(1,0),idynMatrix(1,1),idynMatrix(1,2),
                                idynMatrix(2,0),idynMatrix(2,1),idynMatrix(2,2));
    return true;
}

bool YarptoKDL(const yarp::sig::Vector & yarpVector, KDL::Vector & kdlVector)
{
    if( yarpVector.size() != 3 ) return false;
    memcpy(kdlVector.data,yarpVector.data(),3*sizeof(double));
    return true;
}

bool YarptoKDL(const yarp::sig::Vector & yarpVector, KDL::JntArray & kdlJntArray)
{
    size_t nj = yarpVector.size();
    if( kdlJntArray.rows() != nj ) { kdlJntArray.resize(nj); }
    memcpy(kdlJntArray.data.data(),yarpVector.data(),nj*sizeof(double));
    return true;
}

bool YarptoKDL(const yarp::sig::Vector & yarpVector, KDL::Wrench & kdlWrench)
{
    if( yarpVector.size() != 6 ) return false;
    memcpy(kdlWrench.force.data,yarpVector.data(),3*sizeof(double));
    memcpy(kdlWrench.torque.data,yarpVector.data()+3,3*sizeof(double));
    return true;
}

bool YarptoKDL(const yarp::sig::Vector & yarpVector, KDL::Twist & kdlTwist)
{
    if( yarpVector.size() != 6 ) return false;
    memcpy(kdlTwist.vel.data,yarpVector.data(),3*sizeof(double));
    memcpy(kdlTwist.rot.data,yarpVector.data()+3,3*sizeof(double));
    return true;
}


bool KDLtoYarp(const KDL::Vector & kdlVector,yarp::sig::Vector & yarpVector)
{
    if( yarpVector.size() != 3 ) { yarpVector.resize(3); }
    memcpy(yarpVector.data(),kdlVector.data,3*sizeof(double));
    return true;
}

bool KDLtoYarp(const KDL::Twist & kdlTwist,yarp::sig::Vector & yarpVector)
{
    if( yarpVector.size() != 6 ) { yarpVector.resize(6); }
    memcpy(yarpVector.data(),kdlTwist.vel.data,3*sizeof(double));
    memcpy(yarpVector.data()+3,kdlTwist.rot.data,3*sizeof(double));
    return true;
}

bool KDLtoYarp(const KDL::Wrench & kdlWrench,yarp::sig::Vector & yarpVector)
{
    if( yarpVector.size() != 6 ) { yarpVector.resize(6); }
    memcpy(yarpVector.data(),kdlWrench.force.data,3*sizeof(double));
    memcpy(yarpVector.data()+3,kdlWrench.torque.data,3*sizeof(double));
    return true;
}


yarp::sig::Vector KDLtoYarp(const KDL::Vector & kdlVector)
{
    yarp::sig::Vector yarpVector;
    KDLtoYarp(kdlVector,yarpVector);
    return yarpVector;
}

bool KDLtoYarp(const KDL::JntArray & kdlJntArray,yarp::sig::Vector & yarpVector)
{
    int nj = kdlJntArray.rows();
    if( (int)yarpVector.size() != nj ) { yarpVector.resize(nj); }
    memcpy(yarpVector.data(),kdlJntArray.data.data(),nj*sizeof(double));
    return true;
}

bool KDLtoYarp(const KDL::Rotation & kdlRotation, yarp::sig::Matrix & yarpMatrix3_3)
{
    if( yarpMatrix3_3.rows() != 3 || yarpMatrix3_3.cols() != 3 ) { yarpMatrix3_3.resize(3,3); }
    //Both kdl and yarp store the rotation matrix in row major order
    memcpy(yarpMatrix3_3.data(),kdlRotation.data,3*3*sizeof(double));
    return true;
}

bool KDLtoYarp_position(const KDL::Frame & kdlFrame, yarp::sig::Matrix & yarpMatrix4_4 )
{
    yarp::sig::Matrix R(3,3);
    yarp::sig::Vector p(3);

    KDLtoYarp(kdlFrame.M,R);
    KDLtoYarp(kdlFrame.p,p);
    
    if( yarpMatrix4_4.rows() != 4 || yarpMatrix4_4.cols() != 4 ) { yarpMatrix4_4.resize(4,4); }
    yarpMatrix4_4.zero();
    
    yarpMatrix4_4.setSubmatrix(R,0,0);
    yarpMatrix4_4.setSubcol(p,0,3);
    yarpMatrix4_4(3,3) = 1;
    
    return true;
}

yarp::sig::Matrix KDLtoYarp_position(const KDL::Frame & kdlFrame)
{
    yarp::sig::Matrix yarpMatrix4_4(4,4);
    KDLtoYarp_position(kdlFrame,yarpMatrix4_4);
    return yarpMatrix4_4;
}


bool KDLtoYarp_twist(const KDL::Frame & kdlFrame, yarp::sig::Matrix & yarpMatrix6_6 )
{
    yarp::sig::Matrix R(3,3);
    yarp::sig::Vector p(3);

    KDLtoYarp(kdlFrame.M,R);
    KDLtoYarp(kdlFrame.p,p);
    
    if( yarpMatrix6_6.rows() != 6 || yarpMatrix6_6.cols() != 6 ) { yarpMatrix6_6.resize(6,6); }
    yarpMatrix6_6.zero();
    
    yarpMatrix6_6.setSubmatrix(R,0,0);
    yarpMatrix6_6.setSubmatrix(R,3,3);
    yarpMatrix6_6.setSubmatrix(yarp::math::crossProductMatrix(p)*R,3,0);
    
    return true;
}

yarp::sig::Matrix KDLtoYarp_twist(const KDL::Frame & kdlFrame)
{
    yarp::sig::Matrix yarpMatrix6_6(6,6);
    KDLtoYarp_twist(kdlFrame,yarpMatrix6_6);
    return yarpMatrix6_6;
}


bool KDLtoYarp_wrench(const KDL::Frame &kdlFrame, yarp::sig::Matrix & yarpMatrix6_6 )
{
    yarp::sig::Matrix R(3,3);
    yarp::sig::Vector p(3);

    KDLtoYarp(kdlFrame.M,R);
    KDLtoYarp(kdlFrame.p,p);
    
    if( yarpMatrix6_6.rows() != 6 || yarpMatrix6_6.cols() != 6 ) { yarpMatrix6_6.resize(6,6); }
    yarpMatrix6_6.zero();
    
    yarpMatrix6_6.setSubmatrix(R,0,0);
    yarpMatrix6_6.setSubmatrix(R,3,3);
    yarpMatrix6_6.setSubmatrix(yarp::math::crossProductMatrix(p)*R,0,3);
    
    return true;
}

yarp::sig::Matrix KDLtoYarp_wrench(const KDL::Frame & kdlFrame)
{
    yarp::sig::Matrix yarpMatrix6_6(6,6);
    KDLtoYarp_wrench(kdlFrame,yarpMatrix6_6);
    return yarpMatrix6_6;
}





