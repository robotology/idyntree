// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/SpatialAcc.h>
#include <iDynTree/Transform.h>
#include <iDynTree/Utils.h>
#include <iDynTree/TestUtils.h>
#include <iDynTree/Twist.h>
#include <iDynTree/MatrixFixSize.h>

#include <Eigen/Dense>

#include <cstdio>
#include <cstdlib>

using namespace iDynTree;

void checkInvariance(const Transform & trans, const SpatialAcc & acc1, const SpatialAcc & acc2)
{
    SpatialAcc sumAccTrans = trans*(acc1+acc2);
    SpatialAcc sumAccTransCheck = trans*acc1 + trans*acc2;

    ASSERT_EQUAL_VECTOR(sumAccTrans.asVector(),sumAccTransCheck.asVector());
}

int main()
{
    double accData[6] = {0.4,0.3,-3.0,1.0,2.0,3.0};
    SpatialAcc acc(LinAcceleration(accData,3),AngAcceleration(accData+3,3));

    double acc2Data[6] = {5.4,6.3,-3.0,1.7,2.7,3.0};
    SpatialAcc acc2(LinAcceleration(acc2Data,3),AngAcceleration(acc2Data+3,3));

    Transform trans(Rotation::RPY(5.0,7.0,8.0),Position(10,0,-40));

    checkInvariance(trans,acc,acc2);

    return EXIT_SUCCESS;
}
