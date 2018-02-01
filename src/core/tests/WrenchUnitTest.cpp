/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/Wrench.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/TestUtils.h>

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