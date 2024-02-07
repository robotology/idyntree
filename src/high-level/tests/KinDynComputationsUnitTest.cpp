// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "testModels.h"
#include <iDynTree/TestUtils.h>

#include <iDynTree/Transform.h>
#include <iDynTree/Position.h>
#include <iDynTree/Twist.h>
#include <iDynTree/SpatialAcc.h>
#include <iDynTree/SpatialMomentum.h>
#include <iDynTree/VectorDynSize.h>

#include <iDynTree/EigenHelpers.h>

#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model.h>
#include <iDynTree/Traversal.h>
#include <iDynTree/JointState.h>
#include <iDynTree/FreeFloatingState.h>
#include <iDynTree/SubModel.h>
#include <iDynTree/ModelTransformers.h>

#include <iDynTree/ModelLoader.h>


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

    worldTbase = iDynTree::Transform(Rotation::RPY(random_double(),random_double(),random_double()),
            Position(random_double(),random_double(),random_double()));

    for(int i=0; i < 3; i++)
    {
        gravity(i) = random_double();
    }

    for(int i=0; i < 6; i++)
    {
        baseVel(i) = real_random_double();
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
    iDynTree::Twist avgVel, centroidalAvgVel;
    iDynTree::SpatialMomentum mom, centroidalMom, computedMom, computedCentroidalMom;
    iDynTree::Vector6 avgVelCheck, centroidalAvgVelCheck, momCheck, centroidalMomCheck;
    iDynTree::VectorDynSize nu(dynComp.getNrOfDegreesOfFreedom()+6);
    iDynTree::SpatialInertia lockedInertia, centroidalLockedInertia;
    dynComp.getModelVel(nu);

    MomentumFreeFloatingJacobian momJac(dynComp.getRobotModel());
    MomentumFreeFloatingJacobian centroidalMomJac(dynComp.getRobotModel());
    FrameFreeFloatingJacobian    avgVelJac(dynComp.getRobotModel());
    FrameFreeFloatingJacobian    centroidalAvgVelJac(dynComp.getRobotModel());

    // momentum quantities

    avgVel = dynComp.getAverageVelocity();
    bool ok = dynComp.getAverageVelocityJacobian(avgVelJac);

    ASSERT_IS_TRUE(ok);

    mom = dynComp.getLinearAngularMomentum();
    ok = dynComp.getLinearAngularMomentumJacobian(momJac);

    ASSERT_IS_TRUE(ok);

    lockedInertia = dynComp.getRobotLockedInertia();
    computedMom = lockedInertia * avgVel;

    // centroidal quantities

    centroidalAvgVel = dynComp.getCentroidalAverageVelocity();
    ok = dynComp.getCentroidalAverageVelocityJacobian(centroidalAvgVelJac);

    ASSERT_IS_TRUE(ok);

    centroidalMom = dynComp.getCentroidalTotalMomentum();
    ok = dynComp.getCentroidalTotalMomentumJacobian(centroidalMomJac);
    ASSERT_IS_TRUE(ok);

    centroidalLockedInertia = dynComp.getCentroidalRobotLockedInertia();
    computedCentroidalMom = centroidalLockedInertia * centroidalAvgVel;

    toEigen(momCheck) = toEigen(momJac)*toEigen(nu);
    toEigen(centroidalMomCheck) = toEigen(centroidalMomJac)*toEigen(nu);
    toEigen(avgVelCheck) = toEigen(avgVelJac)*toEigen(nu);
    toEigen(centroidalAvgVelCheck) = toEigen(centroidalAvgVelJac)*toEigen(nu);

    ASSERT_EQUAL_VECTOR(momCheck,mom.asVector());
    ASSERT_EQUAL_VECTOR(centroidalMomCheck,centroidalMom.asVector());

    ASSERT_EQUAL_VECTOR(avgVelCheck,avgVel.asVector());
    ASSERT_EQUAL_VECTOR(centroidalAvgVelCheck, centroidalAvgVel.asVector());

    ASSERT_EQUAL_VECTOR(mom.asVector(), computedMom);
    ASSERT_EQUAL_VECTOR(centroidalMom.asVector(), computedCentroidalMom);
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

inline iDynTree::SpatialAcc Vector6ToSpatialAcc(const iDynTree::Vector6& vec6)
{
    SpatialAcc ret;
    ret(0) = vec6(0);
    ret(1) = vec6(1);
    ret(2) = vec6(2);
    ret(3) = vec6(3);
    ret(4) = vec6(4);
    ret(5) = vec6(5);
    return ret;
}

inline iDynTree::SpatialAcc Vector3ToSpatialAcc(const iDynTree::Vector3& vec6)
{
    SpatialAcc ret;
    ret(0) = vec6(0);
    ret(1) = vec6(1);
    ret(2) = vec6(2);
    ret(3) = 0.0;
    ret(4) = 0.0;
    ret(5) = 0.0;
    return ret;
}

// Test different ways of computing inverse dynamics
void testInverseDynamics(KinDynComputations & dynComp)
{
    int dofs = dynComp.getNrOfDegreesOfFreedom();
    iDynTree::Vector6 baseAcc;
    iDynTree::JointDOFsDoubleArray shapeAccs(dynComp.model());

    iDynTree::LinkNetExternalWrenches netExternalWrenches(dynComp.model());
    for(unsigned int link=0; link < dynComp.model().getNrOfLinks(); link++ )
    {
        netExternalWrenches(link) = getRandomWrench();
    }

    for(int i=0; i < 6; i++)
    {
        baseAcc(i) = real_random_double();
    }

    for(size_t dof=0; dof < dofs; dof++)

    {
        shapeAccs(dof) = random_double();
    }

    {
        FreeFloatingGeneralizedTorques invDynForces(dynComp.model());
        FreeFloatingGeneralizedTorques massMatrixInvDynForces(dynComp.model());
        FreeFloatingGeneralizedTorques regressorInvDynForces(dynComp.model());

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

        FreeFloatingGeneralizedTorques invDynExtForces(dynComp.model());
        ok = dynComp.generalizedExternalForces(netExternalWrenches, invDynExtForces);
        ASSERT_IS_TRUE(ok);

        VectorDynSize massMatrixInvDynForcesContinuous(6+dofs);
        toEigen(massMatrixInvDynForcesContinuous) = toEigen(massMatrix)*toEigen(baseAcc,shapeAccs) + toEigen(invDynBiasForces) + toEigen(invDynExtForces);
        toEigen(massMatrixInvDynForces.baseWrench().getLinearVec3()) = toEigen(massMatrixInvDynForcesContinuous).segment<3>(0);
        toEigen(massMatrixInvDynForces.baseWrench().getAngularVec3()) = toEigen(massMatrixInvDynForcesContinuous).segment<3>(3);
        toEigen(massMatrixInvDynForces.jointTorques()) = toEigen(massMatrixInvDynForcesContinuous).segment(6,dofs);

        ASSERT_EQUAL_SPATIAL_FORCE(massMatrixInvDynForces.baseWrench(),invDynForces.baseWrench());
        ASSERT_EQUAL_VECTOR(massMatrixInvDynForces.jointTorques(),invDynForces.jointTorques());

        // Run inverse dynamics with regressor matrix
        MatrixDynSize regressor(dynComp.model().getNrOfDOFs()+6, 10*dynComp.model().getNrOfLinks());
        ok = dynComp.inverseDynamicsInertialParametersRegressor(baseAcc, shapeAccs, regressor);
        ASSERT_IS_TRUE(ok);

        VectorDynSize inertialParams(10*dynComp.model().getNrOfLinks());
        ok = dynComp.model().getInertialParameters(inertialParams);
        ASSERT_IS_TRUE(ok);

        VectorDynSize regressorInvDynForcesContinuous(6+dofs);
        toEigen(regressorInvDynForcesContinuous) = toEigen(regressor)*toEigen(inertialParams) + toEigen(invDynExtForces);
        toEigen(regressorInvDynForces.baseWrench().getLinearVec3()) = toEigen(regressorInvDynForcesContinuous).segment<3>(0);
        toEigen(regressorInvDynForces.baseWrench().getAngularVec3()) = toEigen(regressorInvDynForcesContinuous).segment<3>(3);
        toEigen(regressorInvDynForces.jointTorques()) = toEigen(regressorInvDynForcesContinuous).segment(6,dofs);

        ASSERT_EQUAL_SPATIAL_FORCE(regressorInvDynForces.baseWrench(),invDynForces.baseWrench());
        ASSERT_EQUAL_VECTOR(regressorInvDynForces.jointTorques(),invDynForces.jointTorques());

        // Run inverse dynamics with regressor matrix with internal joint force/torque
        iDynTree::FreeFloatingGeneralizedTorques invDynForcesWithInternal(dynComp.model());
        iDynTree::LinkInternalWrenches linkInternalWrenches(dynComp.model());

        ok = dynComp.inverseDynamicsWithInternalJointForceTorques(baseAcc,shapeAccs,netExternalWrenches,invDynForcesWithInternal,linkInternalWrenches);
        ASSERT_IS_TRUE(ok);

        // To validate linkInternalWrenches, we use those forces to verify that Newton-Euler equations holds true for each link
        // In particular, we build the three terms of Equation 4.17 in https://traversaro.github.io/traversaro-phd-thesis/traversaro-phd-thesis.pdf:
        //
        // \f$ {}_C \phi_L = {}_C \mathrm{f}^{x}_L + \sum_{D  \in \aleph{L}} {}_C \mathrm{f}_{D,L} \f$
        //
        // inertial "force/torque": \f$ {}_C \phi_L =  \f$ : the term that depend on inertial parameters, including the gravitational force.
        // external force/torque: \f$ {}_C \mathrm{f}^{x}_L \f$ : the net external force/torque that the external environment excerts on the link L.
        // internal force/torque: \f$ \sum_{D  \in \aleph{L}} {}_C \mathrm{f}_{D,L} \f$ : the net (i.e. sum of all) internal force/torque
        //                                           that the other neighbor links (contained in the set $\aleph{L}$) excert on link L via the joints.
        // The main difference is that in Equation 4.17 the quantities are computed in the frame L of the link, while in this test for consistency
        // with the quantities contained in linkInternalWrenches and the rest of results of KinDynComputations, we will use a different frame depending
        // on the getFrameVelocityRepresentation used:
        //
        // |`FrameVelocityRepresentation`     |       C      |
        // |:--------------------------------:|:------------:|
        // | `MIXED_REPRESENTATION` (default) | \f$ L[A] \f$ |
        // | `BODY_FIXED_REPRESENTATION`      | \f$   L  \f$ |
        // | `INERTIAL_FIXED_REPRESENTATION`  | \f$   A \f$  |

        // The inertial term is a complicated to compute in mixed representation, so for now we
        // test only BODY_FIXED_REPRESENTATION or INERTIAL_FIXED_REPRESENTATION
        if (dynComp.getFrameVelocityRepresentation() == MIXED_REPRESENTATION) {
            return;
        }

        iDynTree::LinkWrenches inertialWrenches(dynComp.model());
        iDynTree::LinkWrenches externalWrenches(dynComp.model());
        iDynTree::LinkWrenches internalWrenches(dynComp.model());

        iDynTree::VectorDynSize dummyJointPos, dummyJointVel;
        iDynTree::Vector3 gravityInAbsoluteFrame;
        dummyJointPos.resize(dynComp.model().getNrOfPosCoords());
        dummyJointVel.resize(dynComp.model().getNrOfDOFs());
        dynComp.getRobotState(dummyJointPos, dummyJointVel, gravityInAbsoluteFrame);

        LinkIndex baseLnkIdx = dynComp.getRobotModel().getLinkIndex(dynComp.getFloatingBase());

        for(LinkIndex lnkIdx = 0; lnkIdx < dynComp.getNrOfLinks(); lnkIdx++) {
            // Compute inertialWrenches
            // Inertia is always expressed in L, let's convert it in A if we are in INERTIAL_FIXED_REPRESENTATION
            iDynTree::SpatialInertia rawInertia = dynComp.getRobotModel().getLink(lnkIdx)->getInertia();
            iDynTree::SpatialInertia I;
            if (dynComp.getFrameVelocityRepresentation() == INERTIAL_FIXED_REPRESENTATION) {
                I = dynComp.getWorldTransform(lnkIdx)*rawInertia;
            } else {
                // BODY_FIXED_REPRESENTATION
                I = rawInertia;
            }

            // Not really efficient, but ok as we are in a test
            iDynTree::SpatialAcc     a = Vector6ToSpatialAcc(dynComp.getFrameAcc(lnkIdx, baseAcc,shapeAccs));
            iDynTree::Twist          v = dynComp.getFrameVel(lnkIdx);
            // Add "gravitational" force
            iDynTree::Wrench gravitationalWrench;
            if (dynComp.getFrameVelocityRepresentation() == INERTIAL_FIXED_REPRESENTATION) {
                gravitationalWrench = (I*Vector3ToSpatialAcc(gravityInAbsoluteFrame));
            } else {
                // BODY_FIXED_REPRESENTATION
                gravitationalWrench = (I*(dynComp.getWorldTransform(lnkIdx).inverse()*Vector3ToSpatialAcc(gravityInAbsoluteFrame)));
            }
            inertialWrenches(lnkIdx) = I*a + v*(I*v) - gravitationalWrench;

            // Compute external wrenches (it was already an input of inverseDynamics)
            externalWrenches(lnkIdx) = netExternalWrenches(lnkIdx);

            // Compute internal wrenches
            // We start from the wrench applied by the parent link on the link lnkIdx
            // {}_{C} \mathrm{f}_{\lambda(L), L}
            internalWrenches(lnkIdx) = linkInternalWrenches(lnkIdx);

            // Then we add the link that the link applies on its children
            // rotating them appropriately and inverting their direction
            const iDynTree::Model& model = dynComp.getRobotModel();
            iDynTree::Traversal traversal;
            model.computeFullTreeTraversal(traversal, model.getLinkIndex(dynComp.getFloatingBase()));
            const Link * parentLink	= traversal.getParentLinkFromLinkIndex(lnkIdx);
            for(unsigned int neigh_i=0; neigh_i < model.getNrOfNeighbors(lnkIdx); neigh_i++) {
                LinkIndex neighborIndex = model.getNeighbor(lnkIdx,neigh_i).neighborLink;
                // The child links are the neighbor links that are not parent
                if (!parentLink || neighborIndex != parentLink->getIndex()) {
                    LinkIndex childIndex = neighborIndex;
                    Transform link_X_child;
                    if (dynComp.getFrameVelocityRepresentation() == INERTIAL_FIXED_REPRESENTATION) {
                        // In inertial fixed representation, all joint internal wrenches are expressed in A
                        link_X_child = iDynTree::Transform::Identity();
                    } else {
                        // BODY_FIXED_REPRESENTATION
                        link_X_child = dynComp.getRelativeTransform(lnkIdx,childIndex);
                    }
                    iDynTree::Wrench wrenchAppliedByChildOnLinkInLinkFrame = -(link_X_child*linkInternalWrenches(childIndex));
                    internalWrenches(lnkIdx) = internalWrenches(lnkIdx) + wrenchAppliedByChildOnLinkInLinkFrame;
                }
            }

            // If the link is the base link, we need to add to the external wrenches the baseWrench of the generalized torques
            // This happens because the random-generated external forces and acceleration may not be "consistent" according to
            // definition 4.1 of https://traversaro.github.io/traversaro-phd-thesis/traversaro-phd-thesis.pdf
            if (baseLnkIdx == lnkIdx) {
                externalWrenches(lnkIdx) = externalWrenches(lnkIdx) + invDynForcesWithInternal.baseWrench();
            }

            // Validate Netwon-Euler Equations
            // inertial = external + internal
            ASSERT_EQUAL_SPATIAL_FORCE(inertialWrenches(lnkIdx), externalWrenches(lnkIdx) + internalWrenches(lnkIdx));
        }
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
void testAbsoluteJacobiansAndFrameBiasAcc(KinDynComputations & dynComp)
{
    FrameIndex frame = real_random_int(0, dynComp.getNrOfFrames());

    // Test Jacobian consistency

    // Get robot velocity
    iDynTree::VectorDynSize nu(6+dynComp.getNrOfDegreesOfFreedom());
    dynComp.getModelVel(nu);

    // Compute frame velocity and jacobian
    Twist frameVel = dynComp.getFrameVel(frame);
    Vector6 frameVelJac;
    FrameFreeFloatingJacobian jac(dynComp.model());
    dynComp.getFrameFreeFloatingJacobian(frame, jac);
    toEigen(frameVelJac) = toEigen(jac)*toEigen(nu);

    ASSERT_EQUAL_VECTOR(frameVel.asVector(), frameVelJac);

    std::cerr << "nu: " << nu.toString() << std::endl;

    // Compute frame acceleration, and bias acceleration
    Vector6 baseAcc;
    baseAcc.zero();
    baseAcc(0) = 1;
    baseAcc(1) = 0;
    baseAcc(2) = 0;
    baseAcc(3) = 0;
    baseAcc(4) = 0;
    baseAcc(5) = 0;

    VectorDynSize sddot(dynComp.model().getNrOfDOFs());
    for(int i=0; i < sddot.size(); i++)
    {
        sddot(i) = i*0.1;
    }

    iDynTree::VectorDynSize nuDot(6+dynComp.getNrOfDegreesOfFreedom());
    toEigen(nuDot) = toEigen(baseAcc, sddot);


    Vector6 frameAcc = dynComp.getFrameAcc(frame, baseAcc, sddot);
    Vector6 biasAcc = dynComp.getFrameBiasAcc(frame);
    Vector6 frameAccJac;
    toEigen(frameAccJac) = toEigen(jac)*toEigen(nuDot) + toEigen(biasAcc);

    ASSERT_EQUAL_VECTOR(frameAcc, frameAccJac);
}

void testInverseDynamicsWithInternalJointForceTorques(KinDynComputations & dynComp)
{
    // Comute inverseDynamicsWithInternalJointForceTorques
    iDynTree::LinkInternalWrenches linkInternalWrenches(dynComp.model());

}

bool isStringInVector(const std::string & str,
                      const std::vector<std::string> & vec)
{
    return std::find(vec.begin(), vec.end(), str) != vec.end();
}

void getRandomSubsetOfJoints(const Model & model,
                             size_t nrOfJointsInSubset,
                             std::vector<std::string>& subsetOfJoints)
{
    while( subsetOfJoints.size() < nrOfJointsInSubset )
    {
        JointIndex randomJoint = (JointIndex) getRandomInteger(0,model.getNrOfJoints()-1);
        std::string randomJointName = model.getJointName(randomJoint);

        // If the random added joint is not in the vector, add it
        if( !isStringInVector(randomJointName,subsetOfJoints) )
        {
            subsetOfJoints.push_back(randomJointName);
        }
    }
}

void testSubModelConsistency(std::string modelFilePath, const FrameVelocityRepresentation frameVelRepr)
{
    iDynTree::ModelLoader mdlLoader;
    bool ok = mdlLoader.loadModelFromFile(modelFilePath);
    ASSERT_IS_TRUE(ok);

    iDynTree::KinDynComputations dynCompFullModel;
    ok = dynCompFullModel.loadRobotModel(mdlLoader.model());
    ASSERT_IS_TRUE(ok);

    ok = dynCompFullModel.setFrameVelocityRepresentation(frameVelRepr);
    ASSERT_IS_TRUE(ok);

    setRandomState(dynCompFullModel);

    Model fullModel = mdlLoader.model();

    int dofsFullModel = fullModel.getNrOfDOFs();

    for(size_t jnts=1; jnts < fullModel.getNrOfJoints(); jnts += 5)
    {
        Traversal traversal;
        ok = fullModel.computeFullTreeTraversal(traversal);

        ASSERT_IS_TRUE(ok);

        // Define set of joints which split the model in submodels
        std::vector<std::string> jointInReducedModel;
        getRandomSubsetOfJoints(fullModel, jnts, jointInReducedModel);

        iDynTree::SubModelDecomposition subModels;
        ok = subModels.splitModelAlongJoints(fullModel, traversal, jointInReducedModel);
        ASSERT_IS_TRUE(ok);

        for(int subModelIdx = 0; subModelIdx < subModels.getNrOfSubModels(); subModelIdx++)
        {
            const Traversal & subModelTraversal = subModels.getTraversal(subModelIdx);

            Model reducedModel;
            ok = extractSubModel(fullModel, subModelTraversal, reducedModel);
            ASSERT_IS_TRUE(ok);

            iDynTree::KinDynComputations dynCompReducedModel;
            ok = dynCompReducedModel.loadRobotModel(reducedModel);
            ASSERT_IS_TRUE(ok);

            ok = dynCompReducedModel.setFrameVelocityRepresentation(frameVelRepr);
            ASSERT_IS_TRUE(ok);

            // Get robot state from kinDynFullModel
            iDynTree::VectorDynSize qjFullModel(dofsFullModel), dqjFullModel(dofsFullModel);
            Vector3 gravity;
            Transform    worldTbaseFullModel;
            Twist        baseVelFullModel;
            dynCompFullModel.getRobotState(worldTbaseFullModel, qjFullModel, baseVelFullModel, dqjFullModel, gravity);

            // Get transform between base of full model and base of reduced model
            Transform baseFullTbaseReduced;
            baseFullTbaseReduced = dynCompFullModel.getWorldTransform(subModelTraversal.getLink(0)->getIndex());
            Twist baseVelReducedModel;
            baseVelReducedModel = dynCompFullModel.getFrameVel(subModelTraversal.getLink(0)->getIndex());

            // Find indeces of fullModel joints corresponding to subModel joints
            VectorDynSize idxJntReducedModelInFullModel(reducedModel.getNrOfJoints());

            for(JointIndex jntIdx = 0; jntIdx < reducedModel.getNrOfJoints(); jntIdx++)
            {
                std::string jntName = reducedModel.getJointName(jntIdx);

                idxJntReducedModelInFullModel(jntIdx) = fullModel.getJointIndex(jntName);
            }

            // Get q and dq from kinDynFullModel of joints contained in subModel
            VectorDynSize qReducedModel(reducedModel.getNrOfDOFs()),
                          dqReducedModel(reducedModel.getNrOfDOFs());

            for(JointIndex jntIdx = 0; jntIdx < reducedModel.getNrOfJoints(); jntIdx++)
             {
                size_t posCoordsOffsetReducedModel = reducedModel.getJoint(jntIdx)->getPosCoordsOffset();
                JointIndex jntIdxFullModel = idxJntReducedModelInFullModel(jntIdx);
                size_t posCoordsOffsetFullModel = fullModel.getJoint(jntIdxFullModel)->getPosCoordsOffset();
                for(size_t localPosCoords = 0; localPosCoords < reducedModel.getJoint(jntIdx)->getNrOfPosCoords(); localPosCoords ++)
                {
                    qReducedModel(posCoordsOffsetReducedModel+localPosCoords) = qjFullModel(posCoordsOffsetFullModel+localPosCoords);
                }
             }

            for(JointIndex jntIdx = 0; jntIdx < reducedModel.getNrOfJoints(); jntIdx++)
            {
                size_t dofsOffsetReducedModel = reducedModel.getJoint(jntIdx)->getDOFsOffset();
                JointIndex jntIdxFullModel = idxJntReducedModelInFullModel(jntIdx);
                size_t dofsOffsetFullModel = fullModel.getJoint(jntIdxFullModel)->getDOFsOffset();
                for(size_t localDofs = 0; localDofs < reducedModel.getJoint(jntIdx)->getNrOfDOFs(); localDofs++)
                {
                    dqReducedModel(dofsOffsetReducedModel+localDofs ) = dqjFullModel(dofsOffsetFullModel+localDofs);
                }
            }

            ok = dynCompReducedModel.setRobotState(baseFullTbaseReduced, qReducedModel, baseVelReducedModel, dqReducedModel, gravity);
            ASSERT_IS_TRUE(ok);

            // Compare pose and velocity per each link from full model and from reduced model
            for(FrameIndex frameIdxReducedModel = 0; frameIdxReducedModel < reducedModel.getNrOfFrames(); frameIdxReducedModel++)
            {
                std::string frameName = reducedModel.getFrameName(frameIdxReducedModel);

                int frameIdxFullModel = fullModel.getFrameIndex(frameName);

                // Frame pose from full model
                Transform framePoseFullModel = dynCompFullModel.getWorldTransform(frameIdxFullModel);

                // Frame pose from reduced model
                Transform framePoseReducedModel = dynCompReducedModel.getWorldTransform(frameIdxReducedModel);

                ASSERT_EQUAL_TRANSFORM(framePoseFullModel, framePoseReducedModel);

                // Frame velicity from full model
                Twist frameVelFullModel = dynCompFullModel.getFrameVel(frameName);

                // Frame velocity from reduced model
                Twist frameVelReducedModel = dynCompReducedModel.getFrameVel(frameName);

                ASSERT_EQUAL_VECTOR(frameVelFullModel, frameVelReducedModel);
            }

        }
    }
}

void testModelConsistency(std::string modelFilePath, const FrameVelocityRepresentation frameVelRepr)
{
    iDynTree::KinDynComputations dynComp;
    iDynTree::ModelLoader mdlLoader;
    bool ok = mdlLoader.loadModelFromFile(modelFilePath);
    ok = ok && dynComp.loadRobotModel(mdlLoader.model());
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
        testAbsoluteJacobiansAndFrameBiasAcc(dynComp);
    }

}

void testSubModelConsistencyAllRepresentations(std::string modelName)
{
    std::string urdfFileName = getAbsModelPath(modelName);
    std::cout << "Testing file " << urdfFileName <<  std::endl;
    std::cout << "Testing MIXED_REPRESENTATION " << urdfFileName <<  std::endl;
    testSubModelConsistency(urdfFileName,iDynTree::MIXED_REPRESENTATION);
    std::cout << "Testing BODY_FIXED_REPRESENTATION " << urdfFileName <<  std::endl;
    testSubModelConsistency(urdfFileName,iDynTree::BODY_FIXED_REPRESENTATION);
    std::cout << "Testing INERTIAL_FIXED_REPRESENTATION " << urdfFileName <<  std::endl;
    testSubModelConsistency(urdfFileName,iDynTree::INERTIAL_FIXED_REPRESENTATION);
}

void testModelConsistencyAllRepresentations(std::string modelName)
{
    std::string urdfFileName = getAbsModelPath(modelName);
    std::cout << "Testing file " << urdfFileName <<  std::endl;
    std::cout << "Testing MIXED_REPRESENTATION " << urdfFileName <<  std::endl;
    testModelConsistency(urdfFileName,iDynTree::MIXED_REPRESENTATION);
    std::cout << "Testing BODY_FIXED_REPRESENTATION " << urdfFileName <<  std::endl;
    testModelConsistency(urdfFileName,iDynTree::BODY_FIXED_REPRESENTATION);
    std::cout << "Testing INERTIAL_FIXED_REPRESENTATION " << urdfFileName <<  std::endl;
    testModelConsistency(urdfFileName,iDynTree::INERTIAL_FIXED_REPRESENTATION);
}

void testRelativeJacobianSparsity(KinDynComputations & dynComp)
{
    // take two frames
    if (dynComp.getNrOfLinks() < 2) return;
    FrameIndex frame = -1;
    FrameIndex refFrame = -1;

    if (dynComp.getNrOfLinks() == 2) {
        frame = 0;
        refFrame = 1;
    } else {
        //Pick two frames at random
        frame = real_random_int(0, dynComp.getNrOfFrames());
        //be sure to pick two different frames
        do {
            refFrame = real_random_int(0, dynComp.getNrOfFrames());
        } while (refFrame == frame && frame >= 0);
    }

    // get sparsity pattern
    MatrixDynSize jacobianPattern(6, dynComp.getNrOfDegreesOfFreedom());
    bool ok = dynComp.getRelativeJacobianSparsityPattern(refFrame, frame, jacobianPattern);
    ASSERT_IS_TRUE(ok);

    // Now computes the actual jacobian
    MatrixDynSize jacobian(6, dynComp.getNrOfDegreesOfFreedom());
    ok = dynComp.getRelativeJacobian(refFrame, frame, jacobian);
    ASSERT_IS_TRUE(ok);

    ASSERT_IS_TRUE(jacobian.rows() == jacobianPattern.rows() && jacobian.cols() == jacobian.cols());

    for (size_t row = 0; row < jacobian.rows(); ++row) {
        for (size_t col = 0; col < jacobian.cols(); ++col) {
            // if jacobian has value != 0, pattern should have 1
            if (std::abs(jacobian(row, col)) > iDynTree::DEFAULT_TOL) {
                ASSERT_EQUAL_DOUBLE(jacobianPattern(row, col), 1.0);
            }
        }
    }
}

void testAbsoluteJacobianSparsity(KinDynComputations & dynComp)
{
    // take one frames
    if (dynComp.getNrOfLinks() < 1) return;
    FrameIndex frame = -1;

    if (dynComp.getNrOfLinks() == 1) {
        frame = 0;
    } else {
        //Pick one frames at random
        frame = real_random_int(0, dynComp.getNrOfFrames());
    }

    // get sparsity pattern
    MatrixDynSize jacobianPattern(6, 6 + dynComp.getNrOfDegreesOfFreedom());
    bool ok = dynComp.getFrameFreeFloatingJacobianSparsityPattern(frame, jacobianPattern);
    ASSERT_IS_TRUE(ok);

    // Now computes the actual jacobian
    MatrixDynSize jacobian(6, 6 + dynComp.getNrOfDegreesOfFreedom());
    ok = dynComp.getFrameFreeFloatingJacobian(frame, jacobian);
    ASSERT_IS_TRUE(ok);

    ASSERT_IS_TRUE(jacobian.rows() == jacobianPattern.rows() && jacobian.cols() == jacobian.cols());

    for (size_t row = 0; row < jacobian.rows(); ++row) {
        for (size_t col = 0; col < jacobian.cols(); ++col) {
            // if jacobian has value != 0, pattern should have 1
            if (std::abs(jacobian(row, col)) > iDynTree::DEFAULT_TOL) {
                ASSERT_EQUAL_DOUBLE(jacobianPattern(row, col), 1.0);
            }
        }
    }
}

void testSparsityPattern(std::string modelFilePath, const FrameVelocityRepresentation frameVelRepr)
{
	iDynTree::KinDynComputations dynComp;
    iDynTree::ModelLoader mdlLoader;
    bool ok = mdlLoader.loadModelFromFile(modelFilePath);
    ok = ok && dynComp.loadRobotModel(mdlLoader.model());
    ASSERT_IS_TRUE(ok);

    ok = dynComp.setFrameVelocityRepresentation(frameVelRepr);
    ASSERT_IS_TRUE(ok);

    for(int i=0; i < 10; i++)
    {
        setRandomState(dynComp);
        testRelativeJacobianSparsity(dynComp);
        testAbsoluteJacobianSparsity(dynComp);
    }
}

void testSparsityPatternAllRepresentations(std::string modelName)
{
    std::string urdfFileName = getAbsModelPath(modelName);
    std::cout << "Testing file " << urdfFileName <<  std::endl;
    testSparsityPattern(urdfFileName,iDynTree::MIXED_REPRESENTATION);
    testSparsityPattern(urdfFileName,iDynTree::BODY_FIXED_REPRESENTATION);
    testSparsityPattern(urdfFileName,iDynTree::INERTIAL_FIXED_REPRESENTATION);
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

    testSubModelConsistencyAllRepresentations("oneLink.urdf");
    testSubModelConsistencyAllRepresentations("twoLinks.urdf");
    testSubModelConsistencyAllRepresentations("threeLinks.urdf");
    testSubModelConsistencyAllRepresentations("bigman.urdf");
    testSubModelConsistencyAllRepresentations("icub_skin_frames.urdf");
    testSubModelConsistencyAllRepresentations("iCubGenova02.urdf");
    testSubModelConsistencyAllRepresentations("icalibrate.urdf");

    testSparsityPatternAllRepresentations("oneLink.urdf");
    testSparsityPatternAllRepresentations("twoLinks.urdf");
    testSparsityPatternAllRepresentations("threeLinks.urdf");
    testSparsityPatternAllRepresentations("bigman.urdf");
    testSparsityPatternAllRepresentations("icub_skin_frames.urdf");



    return EXIT_SUCCESS;
}
