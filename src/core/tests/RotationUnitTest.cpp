/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
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

void validateRPYRightTrivializedDerivativeRateOfChange(const Rotation & R, const Vector3 & eulerAngleRateOfChange)
{
    iDynTree::Vector3 rpy = R.asRPY();
    Matrix3x3 DotRPYtoOmegaRateOfChange = Rotation::RPYRightTrivializedDerivativeRateOfChange(rpy(0),rpy(1),rpy(2),
                                                                                              eulerAngleRateOfChange(0), eulerAngleRateOfChange(1),
                                                                                              eulerAngleRateOfChange(2));

    Matrix3x3 OmegaToDotRPYRateOfChange = Rotation::RPYRightTrivializedDerivativeInverseRateOfChange(rpy(0),rpy(1),rpy(2),
                                                                                                     eulerAngleRateOfChange(0), eulerAngleRateOfChange(1),
                                                                                                     eulerAngleRateOfChange(2));

    Matrix3x3 OmegaToDotRPY = Rotation::RPYRightTrivializedDerivativeInverse(rpy(0),rpy(1),rpy(2));
    Matrix3x3 DotRPYToOmega = Rotation::RPYRightTrivializedDerivative(rpy(0),rpy(1),rpy(2));

    Matrix3x3 OmegaToDotRPYRateOfChangeCheck;
    toEigen(OmegaToDotRPYRateOfChangeCheck) = -toEigen(OmegaToDotRPY) * toEigen(DotRPYtoOmegaRateOfChange) * toEigen(OmegaToDotRPY);

    ASSERT_EQUAL_MATRIX(OmegaToDotRPYRateOfChangeCheck, OmegaToDotRPYRateOfChange);

    Vector3 rpyRateOfChangePerturbedUp;
    Vector3 rpyRateOfChangePerturbedDown;
    Vector3 angularVelocity;
    toEigen(angularVelocity) = toEigen(DotRPYToOmega) * toEigen(eulerAngleRateOfChange);

    double numericalDerivStep = 1e-8;

    Vector3 rpyUp;
    Vector3 rpyDown;

    toEigen(rpyUp) = toEigen(rpy) + toEigen(eulerAngleRateOfChange) * numericalDerivStep;
    toEigen(rpyDown) = toEigen(rpy) - toEigen(eulerAngleRateOfChange) * numericalDerivStep;

    Matrix3x3 DotRPYToOmegaUp = Rotation::RPYRightTrivializedDerivative(rpyUp(0),rpyUp(1),rpyUp(2));
    Matrix3x3 DotRPYToOmegaDown = Rotation::RPYRightTrivializedDerivative(rpyDown(0),rpyDown(1),rpyDown(2));

    for(size_t i=0; i < 3; i++)
    {
        rpyRateOfChangePerturbedUp = eulerAngleRateOfChange;
        rpyRateOfChangePerturbedUp(i) = rpyRateOfChangePerturbedUp(i) + numericalDerivStep;
        rpyRateOfChangePerturbedDown = eulerAngleRateOfChange;
        rpyRateOfChangePerturbedDown(i) = rpyRateOfChangePerturbedDown(i) - numericalDerivStep;

        Vector3 RPYNumSecondDev;
        toEigen(RPYNumSecondDev) = (toEigen(rpyRateOfChangePerturbedUp) - toEigen(rpyRateOfChangePerturbedDown))/(2*numericalDerivStep);

        Vector3 angularVelocityUp;
        toEigen(angularVelocityUp) = toEigen(DotRPYToOmegaUp) * toEigen(rpyRateOfChangePerturbedUp);
        Vector3 angularVelocityDown;
        toEigen(angularVelocityDown) = toEigen(DotRPYToOmegaDown) * toEigen(rpyRateOfChangePerturbedDown);

        Vector3 angularAccelerationNum;
        toEigen(angularAccelerationNum) = (toEigen(angularVelocityUp) - toEigen(angularVelocityDown))/(2*numericalDerivStep);

        Vector3 checkDoubleDotRPY;
        toEigen(checkDoubleDotRPY) = toEigen(OmegaToDotRPYRateOfChange) * toEigen(angularVelocity)
            + toEigen(OmegaToDotRPY) * toEigen(angularAccelerationNum);

        ASSERT_EQUAL_VECTOR_TOL(checkDoubleDotRPY,RPYNumSecondDev,1e-4);
    }
}

void validateQuaternionRightTrivializedDerivative(const Rotation & R)
{
    double numericalDerivStep = 1e-8;
    iDynTree::Vector4 quaternion = R.asQuaternion();
    iDynTree::Vector4 quaternionPerturbedUp;
    iDynTree::Vector4 quaternionPerturbedDown;

    MatrixFixSize<4, 3> OmegaToDotQuat = Rotation::QuaternionRightTrivializedDerivative(quaternion);
    MatrixFixSize<3, 4> DotQuatToOmega = Rotation::QuaternionRightTrivializedDerivativeInverse(quaternion);

    Matrix3x3 identityCheck;
    toEigen(identityCheck) = toEigen(DotQuatToOmega) * toEigen(OmegaToDotQuat);

    ASSERT_EQUAL_MATRIX(identityCheck, Rotation::Identity());

    Eigen::Vector4d maxQuaternion;
    maxQuaternion.setConstant(1.0);
    Eigen::Vector4d minQuaternion;
    minQuaternion.setConstant(-1);
    minQuaternion(0) = 0;


    // Test separetly the derivative of quaternion
    for (size_t i = 0; i < 3; i++)
    {
        quaternionPerturbedUp = quaternion;
        quaternionPerturbedUp(i) = quaternionPerturbedUp(i) + numericalDerivStep;
        quaternionPerturbedDown = quaternion;
        quaternionPerturbedDown(i) = quaternionPerturbedDown(i) - numericalDerivStep;
        //ensure validity of obtained quaternion even if the quaternion is no more a step different than
        //the original
        toEigen(quaternionPerturbedUp) = toEigen(quaternionPerturbedUp).array().min(maxQuaternion.array());
        toEigen(quaternionPerturbedUp) = toEigen(quaternionPerturbedUp).array().max(minQuaternion.array());
        toEigen(quaternionPerturbedUp).normalize();

        toEigen(quaternionPerturbedDown) = toEigen(quaternionPerturbedDown).array().min(maxQuaternion.array());
        toEigen(quaternionPerturbedDown) = toEigen(quaternionPerturbedDown).array().max(minQuaternion.array());
        toEigen(quaternionPerturbedDown).normalize();


        Vector4 quatNumDev;
        toEigen(quatNumDev) = (toEigen(quaternionPerturbedUp)-toEigen(quaternionPerturbedDown))/(2*numericalDerivStep);

        Rotation RPerturbedUp = Rotation::RotationFromQuaternion(quaternionPerturbedUp);
        Rotation RPerturbedDown = Rotation::RotationFromQuaternion(quaternionPerturbedDown);

        Matrix3x3 RNumDev;
        toEigen(RNumDev) = (toEigen(RPerturbedUp)-toEigen(RPerturbedDown))/(2*numericalDerivStep);

        Matrix3x3 skewMat;
        toEigen(skewMat) = toEigen(RNumDev)*toEigen(R).transpose();

        Vector3 angVelNum;
        toEigen(angVelNum) = unskew(toEigen(skewMat));

        Vector4 checkDotQuat;
        toEigen(checkDotQuat) = toEigen(OmegaToDotQuat) * toEigen(angVelNum);

        ASSERT_EQUAL_VECTOR_TOL(checkDotQuat,quatNumDev,numericalDerivStep * 10);

        Vector3 angVelCheck;
        toEigen(angVelCheck) = toEigen(DotQuatToOmega) * toEigen(quatNumDev);

        ASSERT_EQUAL_VECTOR_TOL(angVelCheck,angVelNum,numericalDerivStep * 10);

    }
}

void validateAllTests(const Rotation & R, const Vector3 & eulerAngleRateOfChange)
{
    validateRPYRightTrivializedDerivative(R);
    validateQuaternionRightTrivializedDerivative(R);
    validateRPYRightTrivializedDerivativeRateOfChange(R, eulerAngleRateOfChange);
}

int main()
{
    // Check RPYRightTrivializedDerivative

    Rotation R = iDynTree::getRandomRotation();
    Vector3 eulerAngleRateOfChange;
    iDynTree::getRandomVector(eulerAngleRateOfChange, -1.0, 1.0);
    validateAllTests(R, eulerAngleRateOfChange);
    R = iDynTree::getRandomRotation();
    iDynTree::getRandomVector(eulerAngleRateOfChange, -1.0, 1.0);
    validateAllTests(R,eulerAngleRateOfChange);

    return EXIT_SUCCESS;
}
