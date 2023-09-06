// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/Axis.h>
#include <iDynTree/Transform.h>
#include <iDynTree/Utils.h>
#include <iDynTree/TestUtils.h>
#include <iDynTree/SpatialMotionVector.h>

#include <cstdio>
#include <cstdlib>
#include <cmath>

using namespace iDynTree;

void validateExpLogConsistency(const iDynTree::Transform & trans)
{
    iDynTree::Transform transCheck = trans.log().exp();
    ASSERT_EQUAL_TRANSFORM(trans,transCheck);

    std::cerr << "validateExpLogConsistency ok!" << std::endl;
}

void validateLogExpConsistency(const iDynTree::SpatialMotionVector & vec)
{
    std::cout << vec.exp().asHomogeneousTransform().toString() << std::endl;
    iDynTree::SpatialMotionVector vecCheck = vec.exp().log();
    ASSERT_EQUAL_VECTOR(vec,vecCheck);
}

int main()
{
    // test setters and getters
    Transform trans;

    Rotation rot = Rotation::RPY(1,2,3);
    Position  origin(3,2,1);

    trans.setRotation(rot);
    trans.setPosition(origin);

    validateExpLogConsistency(trans);

    SpatialMotionVector vec;
    for(int i=0; i < 6; i++ )
    {
        vec(i) = i/10.0;
    }

    validateLogExpConsistency(vec);

    return EXIT_SUCCESS;
}