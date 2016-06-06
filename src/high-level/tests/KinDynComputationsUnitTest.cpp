/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "testModels.h"
#include <iDynTree/Core/TestUtils.h>

#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Position.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/SpatialAcc.h>
#include <iDynTree/Core/VectorDynSize.h>

#include <iDynTree/KinDynComputations.h>

using namespace iDynTree;

double random_double()
{
    return ((double)rand()-RAND_MAX/2)/((double)RAND_MAX);
}


void setRandomState(iDynTree::KinDynComputations & dynComp)
{
    size_t dofs = dynComp.getNrOfDegreesOfFreedom();
    Transform    worldTbase;
    Twist        baseVel;
    Vector3 gravity;

    iDynTree::VectorDynSize qj(dofs), dqj(dofs), ddqj(dofs);

    worldTbase = //iDynTree::Transform::Identity();
    iDynTree::Transform(Rotation::RPY(random_double(),random_double(),random_double()),
            Position(random_double(),random_double(),random_double()));

    for(int i=0; i < 3; i++)
    {
        gravity(i) = random_double();
    }

    gravity(2) = 10.0;

    for(int i=0; i < 6; i++)
    {
        baseVel(i) = random_double();
    }

    for(size_t dof=0; dof < dofs; dof++)

    {
        qj(dof) = random_double();
        dqj(dof) = random_double();
        ddqj(dof) = random_double();
    }

    bool ok = dynComp.setRobotState(worldTbase,qj,baseVel,dqj,gravity);

    ASSERT_EQUAL_DOUBLE(ok,true);
}


void testRelativeTransform(iDynTree::KinDynComputations & dynComp)
{
    using namespace iDynTree;

    size_t dofs = dynComp.getNrOfDegreesOfFreedom();
    size_t frames = dynComp.getNrOfFrames();

    // Create some frames for random consistency checks
    FrameIndex A = getRandomInteger(0,frames-1);
    FrameIndex B = getRandomInteger(0,frames-1);
    FrameIndex C = getRandomInteger(0,frames-1);
    FrameIndex D = getRandomInteger(0,frames-1);
    FrameIndex E = getRandomInteger(0,frames-1);
    FrameIndex F = getRandomInteger(0,frames-1);

    Transform A_B_X_E_F = dynComp.getRelativeTransformExplicit(A,B,C,D)*dynComp.getRelativeTransformExplicit(C,D,E,F);
    Transform A_B_X_E_F_check = dynComp.getRelativeTransformExplicit(A,B,E,F);
    Transform E_F_X_A_B = dynComp.getRelativeTransformExplicit(E,F,A,B);

    ASSERT_EQUAL_TRANSFORM(A_B_X_E_F,A_B_X_E_F_check);
    ASSERT_EQUAL_TRANSFORM(A_B_X_E_F,E_F_X_A_B.inverse());
}


void testModelConsistency(std::string modelFilePath)
{
    iDynTree::KinDynComputations dynComp;

    bool ok = dynComp.loadRobotModelFromFile(modelFilePath);

    for(int i=0; i < 100; i++)
    {
        setRandomState(dynComp);
        testRelativeTransform(dynComp);
    }
}

int main()
{
    for(unsigned int mdl = 0; mdl < IDYNTREE_TESTS_URDFS_NR; mdl++ )
    {
        std::string urdfFileName = getAbsModelPath(std::string(IDYNTREE_TESTS_URDFS[mdl]));
        std::cout << "Testing file " << std::string(IDYNTREE_TESTS_URDFS[mdl]) <<  std::endl;
        testModelConsistency(urdfFileName);
    }

    return EXIT_SUCCESS;
}
