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
#include <iDynTree/Core/SpatialMomentum.h>
#include <iDynTree/Core/VectorDynSize.h>

#include <iDynTree/Core/EigenHelpers.h>

#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/JointState.h>
#include <iDynTree/Model/FreeFloatingState.h>

using namespace iDynTree;

double random_double()
{
    return 0.0*((double)rand()-RAND_MAX/2)/((double)RAND_MAX);
}

double real_random_double()
{
    return 1.0*((double)rand()-RAND_MAX/2)/((double)RAND_MAX);
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

    gravity(2) = 0.0;

    for(int i=0; i < 6; i++)
    {
        baseVel(i) = i; //real_random_double();
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

void testAverageVelocityAndTotalMomentumJacobian(iDynTree::KinDynComputations & dynComp)
{
    iDynTree::Twist avgVel;
    iDynTree::SpatialMomentum mom;
    iDynTree::Vector6 avgVelCheck, momCheck;
    iDynTree::VectorDynSize nu(dynComp.getNrOfDegreesOfFreedom()+6);
    dynComp.getModelVel(nu);

    MomentumFreeFloatingJacobian momJac(dynComp.getRobotModel());
    FrameFreeFloatingJacobian    avgVelJac(dynComp.getRobotModel());

    avgVel = dynComp.getAverageVelocity();
    bool ok = dynComp.getAverageVelocityJacobian(avgVelJac);

    ASSERT_IS_TRUE(ok);

    mom = dynComp.getLinearAngularMomentum();
    ok = dynComp.getLinearAngularMomentumJacobian(momJac);

    ASSERT_IS_TRUE(ok);

    toEigen(momCheck) = toEigen(momJac)*toEigen(nu);
    toEigen(avgVelCheck) = toEigen(avgVelJac)*toEigen(nu);

    ASSERT_EQUAL_VECTOR(momCheck,mom.asVector());
    ASSERT_EQUAL_VECTOR(avgVelCheck,avgVel.asVector());
}

inline Eigen::VectorXd toEigen(const Vector6 & baseAcc, const VectorDynSize & jntAccs)
{
    Eigen::VectorXd concat(6+jntAccs.size());
    if( jntAccs.size() > 0 )
    {
        concat << toEigen(baseAcc), toEigen(jntAccs);
    }
    else
    {
        concat = toEigen(baseAcc);
    }
    return concat;
}

inline Eigen::VectorXd toEigen(const FreeFloatingGeneralizedTorques & genForces)
{
    Eigen::VectorXd concat(6+genForces.jointTorques().size());
    // TODO(traversaro) : We should teach toEigen to handle empty matrices correctly?
    // relevant: https://forum.kde.org/viewtopic.php?f=74&t=107974
    if( genForces.jointTorques().size() > 0 )
    {
        concat << toEigen(genForces.baseWrench()), toEigen(genForces.jointTorques());
    }
    else
    {
        concat = toEigen(genForces.baseWrench());
    }
    return concat;
}

// Test different ways of computing inverse dynamics
void testInverseDynamics(KinDynComputations & dynComp)
{
    int dofs = dynComp.getNrOfDegreesOfFreedom();
    iDynTree::Vector6 baseAcc;
    iDynTree::JointDOFsDoubleArray shapeAccs(dynComp.model());

    iDynTree::LinkNetExternalWrenches netExternalWrenches(dynComp.model());
    netExternalWrenches.zero();

    // Go component for component, for simplifyng debugging
    for(int i=0; i < 6+dofs; i++)
    {
        baseAcc.zero();
        shapeAccs.zero();
        if( i < 6 )
        {
            baseAcc(i) = 0.0;
        }
        else
        {
            shapeAccs(i-6) = 0.0;
        }

        FreeFloatingGeneralizedTorques invDynForces(dynComp.model());
        FreeFloatingGeneralizedTorques massMatrixInvDynForces(dynComp.model());

        // Run classical inverse dynamics
        bool ok = dynComp.inverseDynamics(baseAcc,shapeAccs,netExternalWrenches,invDynForces);
        ASSERT_IS_TRUE(ok);

        // Run inverse dynamics with mass matrix
        FreeFloatingMassMatrix massMatrix(dynComp.model());
        ok = dynComp.getFreeFloatingMassMatrix(massMatrix);
        ASSERT_IS_TRUE(ok);

        FreeFloatingGeneralizedTorques invDynBiasForces(dynComp.model());
        ok = dynComp.generalizedBiasForces(invDynBiasForces);
        ASSERT_IS_TRUE(ok);

        VectorDynSize massMatrixInvDynForcesContinuous(6+dofs);
        toEigen(massMatrixInvDynForcesContinuous) = toEigen(massMatrix)*toEigen(baseAcc,shapeAccs) + toEigen(invDynBiasForces);
        toEigen(massMatrixInvDynForces.baseWrench().getLinearVec3()) = toEigen(massMatrixInvDynForcesContinuous).segment<3>(0);
        toEigen(massMatrixInvDynForces.baseWrench().getAngularVec3()) = toEigen(massMatrixInvDynForcesContinuous).segment<3>(3);
        toEigen(massMatrixInvDynForces.jointTorques()) = toEigen(massMatrixInvDynForcesContinuous).segment(6,dofs);

        ASSERT_EQUAL_SPATIAL_FORCE(massMatrixInvDynForces.baseWrench(),invDynForces.baseWrench());
        ASSERT_EQUAL_VECTOR(massMatrixInvDynForces.jointTorques(),invDynForces.jointTorques());

    }
}

void testModelConsistency(std::string modelFilePath, const FrameVelocityRepresentation frameVelRepr)
{
    iDynTree::KinDynComputations dynComp;

    bool ok = dynComp.loadRobotModelFromFile(modelFilePath);
    ASSERT_IS_TRUE(ok);

    ok = dynComp.setFrameVelocityRepresentation(frameVelRepr);
    ASSERT_IS_TRUE(ok);

    for(int i=0; i < 5; i++)
    {
        setRandomState(dynComp);
        testRelativeTransform(dynComp);
        testAverageVelocityAndTotalMomentumJacobian(dynComp);
        testInverseDynamics(dynComp);
    }

}

int main()
{
    for(unsigned int mdl = 0; mdl < IDYNTREE_TESTS_URDFS_NR; mdl++ )
    {
        std::string urdfFileName = getAbsModelPath(std::string(IDYNTREE_TESTS_URDFS[mdl]));
        std::cout << "Testing file " << std::string(IDYNTREE_TESTS_URDFS[mdl]) <<  std::endl;
        testModelConsistency(urdfFileName,iDynTree::MIXED_REPRESENTATION);
        testModelConsistency(urdfFileName,iDynTree::BODY_FIXED_REPRESENTATION);
        testModelConsistency(urdfFileName,iDynTree::INERTIAL_FIXED_REPRESENTATION);
    }

    return EXIT_SUCCESS;
}
