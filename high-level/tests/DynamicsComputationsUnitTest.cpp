/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <iDynTree/HighLevel/DynamicsComputations.h>

#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/TestUtils.h>

using namespace iDynTree;

//void checkStateIsDefaultOne(DynamicsComputations & dynComp)
//{
//    dynComp.getStat

//    ASSERT_EQUAL_TRANSFORM(notRotated_H_rotated,);
//}

int main()
{
    DynamicsComputations dynComp;

    bool ok = dynComp.loadRobotModelFromFile("icub.urdf");

    ASSERT_EQUAL_DOUBLE(dynComp.getNrOfDegreesOfFreedom(),32);

//    checkStateIsDefaultOne(dynComp);

    return EXIT_SUCCESS;
}
