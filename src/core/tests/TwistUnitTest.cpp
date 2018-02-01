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
    Transform trans(Rotation::RPY(5.0,7.0,8.0),Position(10,0,-40));

    double twistData[6] = {1.0,4.0,-50.0,1.0,2.0,3.0};
    Twist twist(LinVelocity(twistData,3),AngVelocity(twistData+3,3));

    checkTwistTransformation(trans,twist);

    return EXIT_SUCCESS;
}
