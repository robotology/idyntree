/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
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
    return 1.0*((double)rand()-RAND_MAX/2)/((double)RAND_MAX);
}

double real_random_double()
{
    return 1.0*((double)rand()-RAND_MAX/2)/((double)RAND_MAX);
}

int real_random_int(int initialValue, int finalValue)
{
    int length = finalValue - initialValue;
    return initialValue + rand() % length;
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

    iDynTree::VectorDynSize qj_read(dofs), dqj_read(dofs);
    Vector3 gravity_read;
    Transform    worldTbase_read;
    Twist        baseVel_read;
    dynComp.getRobotState(worldTbase_read, qj_read, baseVel_read, dqj_read, gravity_read);

    ASSERT_EQUAL_VECTOR(qj_read, qj);
    ASSERT_EQUAL_VECTOR(dqj_read, dqj);
    ASSERT_EQUAL_VECTOR(gravity_read, gravity);
    ASSERT_EQUAL_TRANSFORM(worldTbase_read, worldTbase);
    ASSERT_EQUAL_VECTOR(baseVel_read.asVector(), baseVel.asVector());

    ASSERT_EQUAL_DOUBLE(ok,true);
}

void testRelativeTransform(iDynTree::KinDynComputations & dynComp)
{
    using namespace iDynTree;

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
            baseAcc(i) = 1.0;
        }
        else
        {
            shapeAccs(i-6) = 1.0;
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

void testRelativeJacobians(KinDynComputations & dynComp)
{
    if (dynComp.getNrOfLinks() < 2) return;
    FrameIndex frame = -1;
    FrameIndex refFrame = -1;

    if (dynComp.getNrOfLinks() == 2) {
        frame = 0;
        refFrame = 1;
    } else {
        //Pick two frames at random
        frame = real_random_int(0, dynComp.getNrOfFrames());
        refFrame = -1;
        //be sure to pick two different frames
        do {
            refFrame = real_random_int(0, dynComp.getNrOfFrames());
        } while (refFrame == frame && frame >= 0);
    }

    FrameVelocityRepresentation representation = dynComp.getFrameVelocityRepresentation();
    dynComp.setFrameVelocityRepresentation(MIXED_REPRESENTATION);

    //Compute the relative Jacobian
    iDynTree::MatrixDynSize relativeJacobian(6, dynComp.getNrOfDegreesOfFreedom());
    dynComp.getRelativeJacobian(refFrame, frame, relativeJacobian);

    iDynTree::VectorDynSize qj(dynComp.getNrOfDegreesOfFreedom()), dqj(dynComp.getNrOfDegreesOfFreedom());
    Vector3 gravity;
    dynComp.getRobotState(qj, dqj, gravity);

    Twist relativeVel;
    Eigen::Matrix<double, 6, 1> relativeVelTemp;
    relativeVelTemp = toEigen(relativeJacobian) * toEigen(dqj);
    fromEigen(relativeVel, relativeVelTemp);

    //this velocity depends on where the Jacobian is expressed

    Twist frameVel = dynComp.getFrameVel(frame);
    Twist refFrameVel = dynComp.getFrameVel(refFrame);

    if (dynComp.getFrameVelocityRepresentation() == INERTIAL_FIXED_REPRESENTATION) {
        //Inertial = right trivialized.
        //frameVel is written wrt A
        //refFrameVel is written wrt A
        //relativeJacobian is written wrt refFrame
        Eigen::Matrix<double, 6, 1> temp = toEigen(dynComp.getWorldTransform(refFrame).asAdjointTransform()) * toEigen(relativeVel);
        fromEigen(relativeVel, temp);
    } else if (dynComp.getFrameVelocityRepresentation() == BODY_FIXED_REPRESENTATION) {
        //BODY = left trivialized.
        //frameVel is written wrt frame
        //refFrameVel is written wrt refFrame
        //relativeJacobian is written wrt frame
        //convert refFrameVel to frame
        Eigen::Matrix<double, 6, 1> temp = toEigen(dynComp.getRelativeTransform(frame, refFrame).asAdjointTransform()) * toEigen(refFrameVel);
        fromEigen(refFrameVel, temp);
    } else if (dynComp.getFrameVelocityRepresentation() == MIXED_REPRESENTATION) {
        //MIXED
        //frameVel is written wrt frame, [A]
        //refFrameVel is written wrt refFrame, [A]
        //relativeJacobian is written wrt frame, [refFrame]
        //convert refFrameVel to frame, [A]
        Transform frame_A_H_frame(dynComp.getWorldTransform(frame).getRotation(), Position::Zero());

        //refFrameVel = ref_[A]_v_ref, I want frame_[A]_v_ref. As I do not have an explicit A frame I do the following:
        // frame_[A]_H_frame_[frame] * frame_[frame]_H_ref_[frame] * ref_[frame]_H_ref_[A] * ref_[A]_v_ref
        Eigen::Matrix<double, 6, 1> temp = toEigen((frame_A_H_frame * dynComp.getRelativeTransformExplicit(frame, frame, refFrame, frame) * frame_A_H_frame.inverse()).asAdjointTransform()) * toEigen(refFrameVel);
        fromEigen(refFrameVel, temp);
        //and relativeVel to frame [A]
        temp = toEigen((frame_A_H_frame * dynComp.getRelativeTransformExplicit(frame, frame, frame, refFrame)).asAdjointTransform()) * toEigen(relativeVel);
        fromEigen(relativeVel, temp);
    }

    //now compute the error velocity
    //now they should be expressed in the same frame
    Twist velDifference = frameVel - refFrameVel;
    ASSERT_EQUAL_VECTOR(velDifference, relativeVel);

    dynComp.setFrameVelocityRepresentation(representation);
}

// Dummy test: for now it just prints the frameBiasAcc, to check there is no
// usage of not initialized memory
void testFrameBiasAcc(KinDynComputations & dynComp)
{
    FrameIndex frame = real_random_int(0, dynComp.getNrOfFrames());

    // Compute and print frameBiasAcc
    std::cerr << "Computed frameBiasAcc " << dynComp.getFrameBiasAcc(frame).toString() << std::endl;
}

void testModelConsistency(std::string modelFilePath, const FrameVelocityRepresentation frameVelRepr)
{
    iDynTree::KinDynComputations dynComp;

    bool ok = dynComp.loadRobotModelFromFile(modelFilePath);
    ASSERT_IS_TRUE(ok);

    ok = dynComp.setFrameVelocityRepresentation(frameVelRepr);
    ASSERT_IS_TRUE(ok);

    for(int i=0; i < 1; i++)
    {
        setRandomState(dynComp);
        testRelativeTransform(dynComp);
        testAverageVelocityAndTotalMomentumJacobian(dynComp);
        testInverseDynamics(dynComp);
        testRelativeJacobians(dynComp);
        testFrameBiasAcc(dynComp);
    }

}

void testModelConsistencyAllRepresentations(std::string modelName)
{
    std::string urdfFileName = getAbsModelPath(modelName);
    std::cout << "Testing file " << urdfFileName <<  std::endl;
    testModelConsistency(urdfFileName,iDynTree::MIXED_REPRESENTATION);
    testModelConsistency(urdfFileName,iDynTree::BODY_FIXED_REPRESENTATION);
    testModelConsistency(urdfFileName,iDynTree::INERTIAL_FIXED_REPRESENTATION);
}

int main()
{
    // Just run the tests on a handful of models to avoid
    // a long testing time under valgrind
    testModelConsistencyAllRepresentations("oneLink.urdf");
    testModelConsistencyAllRepresentations("twoLinks.urdf");
    testModelConsistencyAllRepresentations("threeLinks.urdf");
    testModelConsistencyAllRepresentations("bigman.urdf");
    testModelConsistencyAllRepresentations("icub_skin_frames.urdf");
    testModelConsistencyAllRepresentations("iCubGenova02.urdf");
    testModelConsistencyAllRepresentations("icalibrate.urdf");

    return EXIT_SUCCESS;
}
