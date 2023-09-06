// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/Axis.h>
#include <iDynTree/Transform.h>
#include <iDynTree/TransformDerivative.h>
#include <iDynTree/Utils.h>
#include <iDynTree/TestUtils.h>
#include <iDynTree/Twist.h>

#include <iDynTree/EigenHelpers.h>

#include <cstdio>
#include <cstdlib>

using namespace iDynTree;

void validateRotationAroundArbitraryAxis(const Axis & ax, const double theta)
{
    Transform notRotated_H_rotated = ax.getRotationTransform(theta);
    Transform notRotated_H_rotated_validation =
        Transform(Rotation::Identity(),ax.getOrigin())
            *Transform(Rotation::RotAxis(ax.getDirection(),theta),Position::Zero())
            *Transform(Rotation::Identity(),-ax.getOrigin());

    assertTransformsAreEqual(notRotated_H_rotated,notRotated_H_rotated_validation);
}

void validateRotationAroundZAxis(const Axis & ax, const double theta)
{
    Transform notRotated_H_rotated = ax.getRotationTransform(theta);
    Transform notRotated_H_rotated_validation =
        Transform(Rotation::Identity(),ax.getOrigin())
            *Transform(Rotation::RotZ(theta),Position())
            *Transform(Rotation::Identity(),-ax.getOrigin());

    assertTransformsAreEqual(notRotated_H_rotated,notRotated_H_rotated_validation);
}

void validateInvarianceOfTwist(const Axis & ax, const Transform & trans, double dtheta)
{
    Twist t = ax.getRotationTwist(dtheta);

    Twist t_trans = trans*t;

    Twist t_trans_check = (trans*ax).getRotationTwist(dtheta);

    ASSERT_EQUAL_SPATIAL_MOTION(t_trans,t_trans_check);
}

Matrix4x4 RotHomTransformNumericalDerivative(const Axis & ax, double theta, double step)
{
    Matrix4x4 ret;

    Matrix4x4 perturbatedUpper = ax.getRotationTransform(theta+step/2).asHomogeneousTransform();
    Matrix4x4 perturbatedLower = ax.getRotationTransform(theta-step/2).asHomogeneousTransform();

    toEigen(ret) = (toEigen(perturbatedUpper)-toEigen(perturbatedLower))/step;

    return ret;
}

Matrix6x6 RotAdjTransformNumericalDerivative(const Axis & ax, double theta, double step)
{
    Matrix6x6 ret;

    Matrix6x6 perturbatedUpper = ax.getRotationTransform(theta+step/2).asAdjointTransform();
    Matrix6x6 perturbatedLower = ax.getRotationTransform(theta-step/2).asAdjointTransform();

    toEigen(ret) = (toEigen(perturbatedUpper)-toEigen(perturbatedLower))/step;

    return ret;
}

Matrix6x6 RotAdjWrenchTransformNumericalDerivative(const Axis & ax, double theta, double step)
{
    Matrix6x6 ret;

    Matrix6x6 perturbatedUpper = ax.getRotationTransform(theta+step/2).asAdjointTransformWrench();
    Matrix6x6 perturbatedLower = ax.getRotationTransform(theta-step/2).asAdjointTransformWrench();

    toEigen(ret) = (toEigen(perturbatedUpper)-toEigen(perturbatedLower))/step;

    return ret;
}

void validateRotationTransformDerivative(const Axis & ax, double theta)
{
    double numericalDerivStep = 1e-8;
    double tol = numericalDerivStep*1e2;

    Transform           trans    = ax.getRotationTransform(theta);
    TransformDerivative analytic = ax.getRotationTransformDerivative(theta);

    Matrix4x4 homTransformDerivAn = analytic.asHomogeneousTransformDerivative();
    Matrix4x4 homTransformDerivNum = RotHomTransformNumericalDerivative(ax,theta,numericalDerivStep);

    Matrix6x6 adjTransformDerivAn = analytic.asAdjointTransformDerivative(trans);
    Matrix6x6 adjTransformDerivNum = RotAdjTransformNumericalDerivative(ax,theta,numericalDerivStep);

    Matrix6x6 adjWrenchTransformDerivAn = analytic.asAdjointTransformWrenchDerivative(trans);
    Matrix6x6 adjWrenchTransformDerivNum = RotAdjWrenchTransformNumericalDerivative(ax,theta,numericalDerivStep);

    std::cout << adjWrenchTransformDerivAn.toString() << std::endl;
    std::cout << adjWrenchTransformDerivNum.toString() << std::endl;


    ASSERT_EQUAL_MATRIX_TOL(homTransformDerivAn,homTransformDerivNum,tol);
    ASSERT_EQUAL_MATRIX_TOL(adjTransformDerivAn,adjTransformDerivNum,tol);
    ASSERT_EQUAL_MATRIX_TOL(adjWrenchTransformDerivAn,adjWrenchTransformDerivNum,tol);
}

void validateAllTests(const Axis ax, const double angle)
{
    validateRotationAroundArbitraryAxis(ax,angle);
    validateRotationTransformDerivative(ax,angle);
}

int main()
{
    // test setters and getters
    Axis ax;

    Direction dir(1,2,3);
    Position  origin(3,2,1);

    ax.setDirection(dir);
    ax.setOrigin(origin);

    ASSERT_EQUAL_VECTOR(ax.getDirection(),dir);
    ASSERT_EQUAL_VECTOR(ax.getOrigin(),origin);

    // test transforms
    Transform trans(Rotation::RPY(0.5,0.4,-0.3),Position(3,4,5));

    ASSERT_EQUAL_VECTOR((trans*ax).getDirection(),trans*dir);
    ASSERT_EQUAL_VECTOR((trans*ax).getOrigin(),trans*origin);

    // Test rotation around the axis
    Axis axRot;

    Direction dirRot(0,0,1);
    Position  originRot(1,0,0);

    axRot.setDirection(dirRot);
    axRot.setOrigin(originRot);

    printf("Validate rotation around arbitrary axis\n");
    validateRotationAroundArbitraryAxis(ax,2.0);
    /*
    printf("Validate rotation around Z axis\n");
    validateRotationAroundZAxis(axRot,4.0);
    printf("Validate invariance of twist of rotation around this axis");
    validateInvarianceOfTwist(ax,trans,1.0);
    validateInvarianceOfTwist(axRot,trans,1.0);*/

    // Stress test
    for(size_t test=0; test < 20; test++ )
    {
        validateAllTests(getRandomAxis(),getRandomDouble());
    }

    return EXIT_SUCCESS;
}