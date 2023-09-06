// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/Twist.h>
#include <iDynTree/Transform.h>
#include <iDynTree/TestUtils.h>

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
    Transform trans(Rotation::RPY(5.0,7.0,8.0),Position(10,0,-40));

    double twistData[6] = {1.0,4.0,-50.0,1.0,2.0,3.0};
    Twist twist(LinVelocity(twistData,3),AngVelocity(twistData+3,3));

    checkTwistTransformation(trans,twist);

    return EXIT_SUCCESS;
}
