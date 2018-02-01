/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#include <iDynTree/Core/Axis.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/SpatialMotionVector.h>

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