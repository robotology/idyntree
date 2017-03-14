/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/Rotation.h>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/TestUtils.h>

#include <cstdio>
#include <cstdlib>

using namespace iDynTree;

void validateRPYRightTrivializedDerivative(const Rotation & R)
{
    double numericalDerivStep = 1e-8;
    iDynTree::Vector3 rpy = R.asRPY();
    iDynTree::Vector3 rpyPerturbedUp;
    iDynTree::Vector3 rpyPerturbedDown;
    
    Matrix3x3 OmegaToDotRPY = Rotation::RPYRightTrivializedDerivativeInverse(rpy(0),rpy(1),rpy(2));
    Matrix3x3 DotRPYToOmega = Rotation::RPYRightTrivializedDerivative(rpy(0),rpy(1),rpy(2));

    Matrix3x3 identityCheck;
    toEigen(identityCheck) = toEigen(OmegaToDotRPY)*toEigen(DotRPYToOmega);
    
    ASSERT_EQUAL_MATRIX(identityCheck,Rotation::Identity());
    
    // Test separetly the derivative of roll, pitch, yaw 
    for(size_t i=0; i < 2; i++)
    {
        rpyPerturbedUp = rpy;
        rpyPerturbedUp(i) = rpyPerturbedUp(i) + numericalDerivStep;
        rpyPerturbedDown = rpy;
        rpyPerturbedDown(i) = rpyPerturbedDown(i) - numericalDerivStep;
        
        Vector3 RPYNumDev;
        toEigen(RPYNumDev) = (toEigen(rpyPerturbedUp)-toEigen(rpyPerturbedDown))/(2*numericalDerivStep);
        
        Rotation RPerturbedUp = Rotation::RPY(rpyPerturbedUp(0),rpyPerturbedUp(1),rpyPerturbedUp(2));
        Rotation RPerturbedDown = Rotation::RPY(rpyPerturbedDown(0),rpyPerturbedDown(1),rpyPerturbedDown(2));

        Matrix3x3 RNumDev;
        toEigen(RNumDev) = (toEigen(RPerturbedUp)-toEigen(RPerturbedDown))/(2*numericalDerivStep);
        
        Matrix3x3 skewMat;
        toEigen(skewMat) = toEigen(RNumDev)*toEigen(R).transpose();
        
        Vector3 angVelNum;
        toEigen(angVelNum) = unskew(toEigen(skewMat));
        
        Vector3 checkDotRPY;
        toEigen(checkDotRPY) = toEigen(OmegaToDotRPY)*toEigen(angVelNum); 
        
        ASSERT_EQUAL_VECTOR_TOL(checkDotRPY,RPYNumDev,numericalDerivStep);
    }
}

void validateAllTests(const Rotation & R)
{
    validateRPYRightTrivializedDerivative(R);
}

int main()
{
    // Check RPYRightTrivializedDerivative 

    Rotation R = iDynTree::getRandomRotation();
    validateAllTests(R);
    R = iDynTree::getRandomRotation();
    validateAllTests(R);
    
    return EXIT_SUCCESS;
}