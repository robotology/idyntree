// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/Wrench.h>
#include <iDynTree/Twist.h>
#include <iDynTree/Transform.h>
#include <iDynTree/TestUtils.h>

#include <Eigen/Dense>

#include <cstdio>
#include <cstdlib>

using namespace iDynTree;

void checkWrenchTransformation(const Transform & trans, const Wrench & w)
{
    Wrench wTransformed = trans*w;
    Matrix6x6 adjWrench = trans.asAdjointTransformWrench();
    Vector6 wTranslatedCheck;

    Vector6 wp = w.asVector();
    toEigen(wTranslatedCheck) = toEigen(adjWrench)*toEigen(wp);

    ASSERT_EQUAL_VECTOR(wTranslatedCheck,wTransformed.asVector());
}

void checkDotProductInvariance(const Transform & trans, const Wrench & w, const Twist & twist)
{
    double power = w.dot(twist);
    double powerCheck = twist.dot(w);
    double powerCheck2 = (trans*w).dot(trans*twist);
    double powerCheck3 = (trans*twist).dot(trans*w);

    ASSERT_EQUAL_DOUBLE(power,powerCheck);
    ASSERT_EQUAL_DOUBLE(power,powerCheck2);
    ASSERT_EQUAL_DOUBLE(power,powerCheck3);

}

int main()
{
    Transform trans(Rotation::RPY(0.0,0.0,0.0),Position(10,0,0));

    double wData[6] = {1.0,4.0,-50.0,1.0,2.0,3.0};
    Wrench wrench(LinearForceVector3(wData,3),AngularForceVector3(wData+3,3));

    double twistData[6] = {14.0,-4.0,-5.0,6.0,7.0,1.0};
    Twist twist(LinVelocity(twistData,3),AngVelocity(twistData+3,3));

    checkWrenchTransformation(trans,wrench);
    checkDotProductInvariance(trans,wrench,twist);

    return EXIT_SUCCESS;
}