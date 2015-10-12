/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "testModels.h"

#include <iDynTree/HighLevel/DynamicsComputations.h>

#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/SpatialAcc.h>
#include <iDynTree/Core/ClassicalAcc.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/Utils.h>
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/Wrench.h>

#include <iDynTree/Core/EigenHelpers.h>

using namespace iDynTree;

//void checkStateIsDefaultOne(DynamicsComputations & dynComp)
//{
//    dynComp.getStat

//    ASSERT_EQUAL_TRANSFORM(notRotated_H_rotated,);
//}

double random_double()
{
    return ((double)rand()-RAND_MAX/2)/((double)RAND_MAX);
}


void setRandomState(iDynTree::HighLevel::DynamicsComputations & dynComp)
{
    size_t dofs = dynComp.getNrOfDegreesOfFreedom();
    Transform    worldTbase;
    Twist        baseVel;
    ClassicalAcc baseAcc;
    SpatialAcc gravity;
    Vector6    properAcc;

    iDynTree::VectorDynSize qj(dofs), dqj(dofs), ddqj(dofs);

    worldTbase = iDynTree::Transform::Identity();
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
        baseAcc(i) = random_double();
        properAcc(i) = baseAcc(i) + gravity(i);
    }

    for(int dof=0; dof < dofs; dof++)

    {
        qj(dof) = random_double();
        dqj(dof) = random_double();
        ddqj(dof) = random_double();
    }

    bool ok = dynComp.setRobotState(qj,dqj,ddqj,worldTbase,baseVel,baseAcc,gravity);

    ASSERT_EQUAL_DOUBLE(ok,true);
}

void testRegressorVsInverseDynamicsConsistency(iDynTree::HighLevel::DynamicsComputations & dynComp)
{
    size_t dofs = dynComp.getNrOfDegreesOfFreedom();
    size_t nrOfLinks = dynComp.getNrOfLinks();

    // check regressor matrix
    MatrixDynSize dynRegressor(6+dofs,10*nrOfLinks);
    dynComp.getDynamicsRegressor(dynRegressor);

    // check parameter vector
    VectorDynSize dynParams(10*nrOfLinks);
    dynComp.getModelDynamicsParameters(dynParams);

    // compute inverseDynamics with regressor
    VectorDynSize idResultRegr(6+dofs), torquesRegr(dofs), torquesID(dofs);
    Wrench baseWrenchID, baseWrenchRegr;
    toEigen(idResultRegr) = toEigen(dynRegressor)*toEigen(dynParams);

    toEigen(baseWrenchRegr.getLinearVec3()) = toEigen(idResultRegr).segment(0,3);
    toEigen(baseWrenchRegr.getAngularVec3()) = toEigen(idResultRegr).segment(3,3);

    toEigen(torquesRegr)    = toEigen(idResultRegr).segment(6,dofs);

    // compute inverseDynam
    dynComp.inverseDynamics(torquesID,baseWrenchID);

    ASSERT_EQUAL_VECTOR(torquesID,torquesRegr);
    ASSERT_EQUAL_VECTOR(baseWrenchID,baseWrenchRegr);
}

void testModelConsistency(std::string modelFilePath)
{
    HighLevel::DynamicsComputations dynComp;

    bool ok = dynComp.loadRobotModelFromFile(modelFilePath);

    setRandomState(dynComp);

    testRegressorVsInverseDynamicsConsistency(dynComp);
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
