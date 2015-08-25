/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/IMatrix.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/TestUtils.h>

#include <Eigen/Dense>

#include <cstdio>
#include <cstdlib>

using namespace iDynTree;

void checkTwistTransformation(const Transform & trans, const Twist & twist)
{
    Twist twistTransformed = trans*twist;
    Matrix6x6 adj = trans.asAdjointTransform();
    Vector6 twistTranslatedCheck;

    Vector6 twistp = twist.asVector();
    toEigen(twistTranslatedCheck) = toEigen(adj)*toEigen(twistp);

    ASSERT_EQUAL_VECTOR(twistTranslatedCheck,twistTransformed.asVector());
}

int main()
{
    Transform trans(Rotation::RPY(0.0,0.0,0.0),Position(10,0,0));

    double twistData[6] = {1.0,4.0,-50.0,1.0,2.0,3.0};
    Twist twist(LinVelocity(twistData,3),AngVelocity(twistData+3,3));

    checkTwistTransformation(trans,twist);

    return EXIT_SUCCESS;
}