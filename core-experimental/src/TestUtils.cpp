/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/SpatialForceVector.h>
#include <iDynTree/Core/SpatialMotionVector.h>
#include <iDynTree/Core/Axis.h>
#include <iDynTree/Core/SpatialInertia.h>
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/Transform.h>

#include <iostream>

#include <cstdlib>
#include <cmath>

namespace iDynTree
{

void assertStringAreEqual(const std::string& val1, const std::string& val2, double tol, std::string file, int line)
{
    if( val1 != val2 )
    {
       std::cerr << "assertStringAreEqual failure: val1 is " << val1
                  << " while val2 is " << val2 << std::endl;
            exit(EXIT_FAILURE);
    }
}

void assertDoubleAreEqual(const double& val1, const double& val2, double tol, std::string file, int line)
{
    if( fabs(val1-val2) >= tol )
    {
       std::cerr << "assertDoubleAreEqual failure: val1 is " << val1
                  << " while val2 is " << val2 << std::endl;
            exit(EXIT_FAILURE);
    }
}

void printVector(std::string name, const IVector& vec)
{
    std::cerr << name << " : \n";
    for(int i=0; i < vec.size(); i++ )
    {
        std::cerr << vec(i) << "\n";
    }
}

void assertVectorAreEqual(const IVector& vec1, const IVector& vec2, double tol, std::string file, int line)
{
    if( vec1.size() != vec2.size() )
    {
        std::cerr << file << ":" << line << " : assertVectorAreEqual failure: vec1 has size " << vec1.size()
                  << " while vec2 has size " << vec2.size() << std::endl;
        exit(EXIT_FAILURE);
    }

    for( unsigned int i = 0; i < vec1.size(); i++ )
    {
        if( fabs(vec1(i)-vec2(i)) >= tol )
        {
            std::cerr << file << ":" << line << " : assertVectorAreEqual failure: element " << i << " of vec1 is " << vec1(i)
                  << " while of vec2 is " << vec2(i) << std::endl;
            printVector("vec1",vec1);
            printVector("vec2",vec2);
            exit(EXIT_FAILURE);
        }
    }
}

void assertMatrixAreEqual(const IMatrix& mat1, const IMatrix& mat2, double tol, std::string file, int line)
{
    if( mat1.rows() != mat2.rows() ||
        mat2.cols() != mat1.cols() )
    {
        std::cerr << file << ":" << line << " : assertMatrixAreEqual failure: mat1 has size " << mat1.rows() << " " << mat1.cols()
                  << " while mat2 has size " << mat2.rows() << " " << mat2.cols() << std::endl;
        exit(EXIT_FAILURE);
    }

    for( unsigned int row = 0; row < mat2.rows(); row++ )
    {
        for( unsigned int col = 0; col < mat2.cols(); col++ )
        {
            if( fabs(mat1(row,col)-mat2(row,col)) >= tol )
            {
                std::cerr << file << ":" << line << " : assertMatrixAreEqual failure: element " << row << " " << col << " of mat1 is " << mat1(row,col)
                  << " while of mat2 is " << mat2(row,col) << std::endl;
                exit(EXIT_FAILURE);
            }
        }
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

double getRandomDouble(double min, double max)
{
    return min + (max-min)*((double)rand())/((double)RAND_MAX);
}

Position getRandomPosition()
{
    return Position(getRandomDouble(-2,2),getRandomDouble(-2,2),getRandomDouble(-2,2));
}

Rotation getRandomRotation()
{
    return Rotation::RPY(getRandomDouble(),getRandomDouble(-1,1),getRandomDouble());
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
                               rot*RotationalInertiaRaw(rotInertiaData,3,3));
}




}