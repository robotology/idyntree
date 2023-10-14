// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/SpatialForceVector.h>
#include <iDynTree/SpatialMotionVector.h>
#include <iDynTree/Axis.h>
#include <iDynTree/SpatialForceVector.h>
#include <iDynTree/SpatialMotionVector.h>
#include <iDynTree/SpatialInertia.h>
#include <iDynTree/TestUtils.h>
#include <iDynTree/Transform.h>

#include <iostream>

#include <cstdlib>
#include <cmath>

namespace iDynTree
{

void assertStringAreEqual(const std::string& val1, const std::string& val2, double /*tol*/, std::string file, int line)
{
    if( val1 != val2 )
    {
       std::cerr << file << ":" << line << " : assertStringAreEqual failure: val1 is " << val1
                  << " while val2 is " << val2 << std::endl;
            exit(EXIT_FAILURE);
    }
}

void assertTrue(bool prop, std::string file, int line)
{
    if( !prop )
    {
       std::cerr << file << ":" << line << " : assertTrue failure" << std::endl;
       exit(EXIT_FAILURE);
    }
}


void assertDoubleAreEqual(const double& val1, const double& val2, double tol, std::string file, int line)
{
    if( !(fabs(val1-val2) < tol) )
    {
       std::cerr << file << ":" << line << " : assertDoubleAreEqual failure: val1 is " << val1
                  << " while val2 is " << val2 << std::endl;
            exit(EXIT_FAILURE);
    }
}

void assertTransformsAreEqual(const Transform& trans1, const Transform& trans2, double tol, std::string file, int line)
{
    assertVectorAreEqual(trans1.getPosition(),trans2.getPosition(),tol,file,line);
    assertMatrixAreEqual(trans1.getRotation(),trans2.getRotation(),tol,file,line);
}

void assertSpatialForceAreEqual(const SpatialForceVector& f1, const SpatialForceVector& f2, double tol, std::string file, int line)
{
    Vector6 f1plain = f1.asVector();
    Vector6 f2plain = f2.asVector();
    assertVectorAreEqual(f1plain,f2plain,tol,file,line);
}

void assertSpatialMotionAreEqual(const SpatialMotionVector& f1, const SpatialMotionVector& f2, double tol, std::string file, int line)
{
    Vector6 f1plain = f1.asVector();
    Vector6 f2plain = f2.asVector();
    assertVectorAreEqual(f1plain,f2plain,tol,file,line);
}

bool getRandomBool()
{
    double coin = getRandomDouble(0,1.0);
    return (coin >= 0.5);
}

double getRandomDouble(double min, double max)
{
    return min + (max-min)*((double)rand())/((double)RAND_MAX);
}

int getRandomInteger(int min, int max)
{
    return (rand() % (max-min+1)) + min;
}

Position getRandomPosition()
{
    return Position(getRandomDouble(-2,2),getRandomDouble(-2,2),getRandomDouble(-2,2));
}

Rotation getRandomRotation()
{
    return Rotation::RPY(getRandomDouble(-10,10),getRandomDouble(-10,10),getRandomDouble(-10,10));
}

Transform getRandomTransform()
{
    return Transform(getRandomRotation(),getRandomPosition());
}


Axis getRandomAxis()
{
    return Axis(getRandomRotation()*Direction(1,0,0),getRandomPosition());
}


SpatialInertia getRandomInertia()
{
    double cxx = getRandomDouble(0,3);
    double cyy = getRandomDouble(0,4);
    double czz = getRandomDouble(0,6);
    double rotInertiaData[3*3] = {czz+cyy,0.0,0.0,
                                  0.0,cxx+czz,0.0,
                                  0.0,0.0,cxx+cyy};

    Rotation rot = Rotation::RPY(getRandomDouble(),getRandomDouble(-1,1),getRandomDouble());

    SpatialInertia inertiaLink(getRandomDouble(0,4),
                               Position(getRandomDouble(-2,2),getRandomDouble(-2,2),getRandomDouble(-2,2)),
                               rot*RotationalInertia(rotInertiaData,3,3));

    return inertiaLink;
}

SpatialMotionVector getRandomTwist()
{
    SpatialMotionVector ret;

    for(int i=0; i < 3; i++ )
    {
        ret.getLinearVec3()(i) = getRandomDouble();
        ret.getAngularVec3()(i) = getRandomDouble();
    }

    return ret;
}

SpatialForceVector getRandomWrench()
{
    SpatialForceVector ret;

    for(int i=0; i < 3; i++ )
    {
        ret.getLinearVec3()(i) = getRandomDouble();
        ret.getAngularVec3()(i) = getRandomDouble();
    }

    return ret;
}





}
