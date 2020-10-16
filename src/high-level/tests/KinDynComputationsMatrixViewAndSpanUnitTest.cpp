/*
 * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia
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
#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/JointState.h>
#include <iDynTree/Model/FreeFloatingState.h>

#include <iDynTree/ModelIO/ModelLoader.h>


namespace Eigen
{
    using Vector6d = Eigen::Matrix<double, 6, 1>;
}

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
    Eigen::Vector6d baseVel;
    Eigen::Vector3d gravity;

    Eigen::VectorXd qj(dofs), dqj(dofs), ddqj(dofs);

    worldTbase = iDynTree::Transform(Rotation::RPY(random_double(),random_double(),random_double()),
                                     Position(random_double(),random_double(),random_double()));


    Eigen::Matrix4d transform = toEigen(worldTbase.asHomogeneousTransform());

    for(int i=0; i < 3; i++)
    {
        gravity(i) = random_double();
    }

    gravity(2) = 0.0;

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

    bool ok = dynComp.setRobotState(transform,
                                    make_span(qj.data(), qj.size()),
                                    make_span(baseVel.data(), baseVel.size()),
                                    make_span(dqj.data(), dqj.size()),
                                    make_span(gravity.data(), gravity.size()));


    ASSERT_IS_TRUE(ok);

    Eigen::VectorXd qj_read(dofs), dqj_read(dofs);
    Eigen::Vector3d gravity_read;
    Eigen::Matrix4d transform_read;
    Eigen::Vector6d baseVel_read;

    ok = dynComp.getRobotState(transform_read,
                               make_span(qj_read.data(), qj_read.size()),
                               make_span(baseVel_read.data(), baseVel_read.size()),
                               make_span(dqj_read.data(), dqj_read.size()),
                               make_span(gravity_read.data(), gravity_read.size()));

    ASSERT_IS_TRUE(ok);

    ASSERT_EQUAL_VECTOR(qj_read, qj);
    ASSERT_EQUAL_VECTOR(dqj_read, dqj);
    ASSERT_EQUAL_VECTOR(gravity_read, gravity);
    ASSERT_EQUAL_MATRIX(transform_read, transform);
    ASSERT_EQUAL_VECTOR(baseVel_read, baseVel);

    ASSERT_EQUAL_DOUBLE(ok,true);
}

void testAverageVelocityAndTotalMomentumJacobian(iDynTree::KinDynComputations & dynComp)
{
    Eigen::Vector6d avgVel;
    Eigen::Vector6d mom, centroidalMom;
    Eigen::Vector6d avgVelCheck, momCheck, centroidalMomCheck;
    Eigen::VectorXd nu(dynComp.getNrOfDegreesOfFreedom()+6);
    ASSERT_IS_TRUE(dynComp.getModelVel(make_span(nu.data(), nu.size())));

    Eigen::MatrixXd momJac(6, dynComp.getNrOfDegreesOfFreedom()+6);
    Eigen::MatrixXd centroidalMomJac(6, dynComp.getNrOfDegreesOfFreedom()+6);
    Eigen::MatrixXd avgVelJac(6, dynComp.getNrOfDegreesOfFreedom()+6);


    bool ok = dynComp.getAverageVelocity(make_span(avgVel.data(), avgVel.size()));
    ASSERT_IS_TRUE(ok);

    ok = dynComp.getAverageVelocityJacobian(avgVelJac);
    ASSERT_IS_TRUE(ok);

    ok = dynComp.getLinearAngularMomentum(make_span(mom.data(), mom.size()));
    ASSERT_IS_TRUE(ok);

    ok = dynComp.getLinearAngularMomentumJacobian(momJac);
    ASSERT_IS_TRUE(ok);

    ok = dynComp.getCentroidalTotalMomentum(make_span(centroidalMom.data(), centroidalMom.size()));
    ASSERT_IS_TRUE(ok);

    ok = dynComp.getCentroidalTotalMomentumJacobian(centroidalMomJac);
    ASSERT_IS_TRUE(ok);

    momCheck = momJac * nu;
    centroidalMomCheck = centroidalMomJac * nu;
    avgVelCheck = avgVelJac * nu;

    ASSERT_EQUAL_VECTOR(momCheck, mom);
    ASSERT_EQUAL_VECTOR(centroidalMomCheck, centroidalMom);
    ASSERT_EQUAL_VECTOR(avgVelCheck, avgVel);
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
    Eigen::Vector6d baseAcc;
    Eigen::VectorXd shapeAccs(dynComp.getNrOfDegreesOfFreedom());

    iDynTree::LinkNetExternalWrenches netExternalWrenches(dynComp.model());
    for(unsigned int link=0; link < dynComp.model().getNrOfLinks(); link++ )
    {
        netExternalWrenches(link) = getRandomWrench();
    }

    // Go component for component, for simplifyng debugging
    for(int i=0; i < 6+dofs; i++)
    {
        baseAcc.setZero();
        shapeAccs.setZero();
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
        bool ok = dynComp.inverseDynamics(make_span(baseAcc.data(), baseAcc.size()),
                                          make_span(shapeAccs.data(), shapeAccs.size()),
                                          netExternalWrenches,invDynForces);
        ASSERT_IS_TRUE(ok);

        // Run inverse dynamics with mass matrix
        Eigen::MatrixXd massMatrix(dynComp.getNrOfDegreesOfFreedom() + 6, dynComp.getNrOfDegreesOfFreedom() + 6);
        ok = dynComp.getFreeFloatingMassMatrix(massMatrix);
        ASSERT_IS_TRUE(ok);

        Eigen::VectorXd invDynBiasForces(dynComp.getNrOfDegreesOfFreedom() + 6);
        ok = dynComp.generalizedBiasForces(make_span(invDynBiasForces.data(), invDynBiasForces.size()));
        ASSERT_IS_TRUE(ok);

        FreeFloatingGeneralizedTorques invDynExtForces(dynComp.model());
        ok = dynComp.generalizedExternalForces(netExternalWrenches, invDynExtForces);
        ASSERT_IS_TRUE(ok);

        Eigen::VectorXd massMatrixInvDynForcesContinuous(6+dofs);
        Eigen::VectorXd robotAccs(baseAcc.size() + shapeAccs.size());
        robotAccs << baseAcc, shapeAccs;

        massMatrixInvDynForcesContinuous = massMatrix * robotAccs + invDynBiasForces + toEigen(invDynExtForces);
        toEigen(massMatrixInvDynForces.baseWrench().getLinearVec3()) = massMatrixInvDynForcesContinuous.segment<3>(0);
        toEigen(massMatrixInvDynForces.baseWrench().getAngularVec3()) = massMatrixInvDynForcesContinuous.segment<3>(3);
        toEigen(massMatrixInvDynForces.jointTorques()) = massMatrixInvDynForcesContinuous.segment(6,dofs);

        ASSERT_EQUAL_SPATIAL_FORCE(massMatrixInvDynForces.baseWrench(), invDynForces.baseWrench());
        ASSERT_EQUAL_VECTOR(massMatrixInvDynForces.jointTorques(), invDynForces.jointTorques());
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
    Eigen::MatrixXd relativeJacobian(6, dynComp.getNrOfDegreesOfFreedom());
    bool ok = dynComp.getRelativeJacobian(refFrame, frame, relativeJacobian);
    ASSERT_IS_TRUE(ok);

    Eigen::VectorXd qj(dynComp.getNrOfDegreesOfFreedom()), dqj(dynComp.getNrOfDegreesOfFreedom());
    Eigen::Vector3d gravity;
    dynComp.getRobotState(make_span(qj.data(), qj.size()),
                               make_span(dqj.data(), dqj.size()),
                               make_span(gravity.data(), gravity.size()));

    Eigen::Vector6d relativeVel;
    relativeVel = relativeJacobian * dqj;

    //this velocity depends on where the Jacobian is expressed

    Eigen::Vector6d frameVel, refFrameVel;
    ok = dynComp.getFrameVel(frame, make_span(frameVel.data(), frameVel.size()));
    ASSERT_IS_TRUE(ok);

    ok = dynComp.getFrameVel(refFrame, make_span(refFrameVel.data(), refFrameVel.size()));
    ASSERT_IS_TRUE(ok);

    if (dynComp.getFrameVelocityRepresentation() == INERTIAL_FIXED_REPRESENTATION) {
        //Inertial = right trivialized.
        //frameVel is written wrt A
        //refFrameVel is written wrt A
        //relativeJacobian is written wrt refFrame
        relativeVel = toEigen(dynComp.getWorldTransform(refFrame).asAdjointTransform()) * relativeVel;
    } else if (dynComp.getFrameVelocityRepresentation() == BODY_FIXED_REPRESENTATION) {
        //BODY = left trivialized.
        //frameVel is written wrt frame
        //refFrameVel is written wrt refFrame
        //relativeJacobian is written wrt frame
        //convert refFrameVel to frame
        relativeVel = toEigen(dynComp.getRelativeTransform(frame, refFrame).asAdjointTransform()) * refFrameVel;
    } else if (dynComp.getFrameVelocityRepresentation() == MIXED_REPRESENTATION) {
        //MIXED
        //frameVel is written wrt frame, [A]
        //refFrameVel is written wrt refFrame, [A]
        //relativeJacobian is written wrt frame, [refFrame]
        //convert refFrameVel to frame, [A]
        Transform frame_A_H_frame(dynComp.getWorldTransform(frame).getRotation(), Position::Zero());

        //refFrameVel = ref_[A]_v_ref, I want frame_[A]_v_ref. As I do not have an explicit A frame I do the following:
        // frame_[A]_H_frame_[frame] * frame_[frame]_H_ref_[frame] * ref_[frame]_H_ref_[A] * ref_[A]_v_ref
        Eigen::Vector6d temp = toEigen((frame_A_H_frame * dynComp.getRelativeTransformExplicit(frame, frame, refFrame, frame) * frame_A_H_frame.inverse()).asAdjointTransform()) * refFrameVel;
        refFrameVel = temp;
        //and relativeVel to frame [A]
        temp = toEigen((frame_A_H_frame * dynComp.getRelativeTransformExplicit(frame, frame, frame, refFrame)).asAdjointTransform()) * relativeVel;
        relativeVel = temp;
    }

    //now compute the error velocity
    //now they should be expressed in the same frame
    Eigen::Vector6d velDifference = frameVel - refFrameVel;
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
    Eigen::VectorXd nu(6+dynComp.getNrOfDegreesOfFreedom());
    bool ok = dynComp.getModelVel(make_span(nu.data(), nu.size()));
    ASSERT_IS_TRUE(ok);

    // Compute frame velocity and jacobian
    Eigen::Vector6d frameVel;

    ok = dynComp.getFrameVel(frame, make_span(frameVel.data(), frameVel.size()));
    ASSERT_IS_TRUE(ok);

    Eigen::Vector6d frameVelJac;
    Eigen::MatrixXd jac(6, dynComp.getNrOfDegreesOfFreedom() + 6);
    ok = dynComp.getFrameFreeFloatingJacobian(frame, jac);
    ASSERT_IS_TRUE(ok);

    frameVelJac = jac * nu;

    ASSERT_EQUAL_VECTOR(frameVel, frameVelJac);

    std::cerr << "nu: " << nu.transpose() << std::endl;

    // Compute frame acceleration, and bias acceleration
    Eigen::Vector6d baseAcc;
    baseAcc.setZero();
    baseAcc(0) = 1;
    baseAcc(1) = 0;
    baseAcc(2) = 0;
    baseAcc(3) = 0;
    baseAcc(4) = 0;
    baseAcc(5) = 0;

    Eigen::VectorXd sddot(dynComp.model().getNrOfDOFs());
    for(int i=0; i < sddot.size(); i++)
    {
        sddot(i) = i*0.1;
    }

    Eigen::VectorXd nuDot(6+dynComp.getNrOfDegreesOfFreedom());
    nuDot.head<6>() = baseAcc;
    nuDot.tail(dynComp.getNrOfDegreesOfFreedom()) = sddot;


    Eigen::Vector6d frameAcc, biasAcc, frameAccJac;
    ok = dynComp.getFrameAcc(frame,
                             make_span(baseAcc.data(), baseAcc.size()),
                             make_span(sddot.data(), sddot.size()),
                             make_span(frameAcc.data(), frameAcc.size()));
    ASSERT_IS_TRUE(ok);

    ok = dynComp.getFrameBiasAcc(frame, make_span(biasAcc.data(), biasAcc.size()));
    ASSERT_IS_TRUE(ok);

    frameAccJac = jac * nuDot + biasAcc;

    ASSERT_EQUAL_VECTOR(frameAcc, frameAccJac);
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
        testAverageVelocityAndTotalMomentumJacobian(dynComp);
        testInverseDynamics(dynComp);
        testRelativeJacobians(dynComp);
        testAbsoluteJacobiansAndFrameBiasAcc(dynComp);
    }

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
    Eigen::MatrixXd jacobianPattern(6, dynComp.getNrOfDegreesOfFreedom());
    bool ok = dynComp.getRelativeJacobianSparsityPattern(refFrame, frame, jacobianPattern);
    ASSERT_IS_TRUE(ok);

    // Now computes the actual jacobian
    Eigen::MatrixXd jacobian(6, dynComp.getNrOfDegreesOfFreedom());
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
    Eigen::MatrixXd jacobianPattern(6, 6 + dynComp.getNrOfDegreesOfFreedom());
    bool ok = dynComp.getFrameFreeFloatingJacobianSparsityPattern(frame, jacobianPattern);
    ASSERT_IS_TRUE(ok);

    // Now computes the actual jacobian
    Eigen::MatrixXd jacobian(6, 6 + dynComp.getNrOfDegreesOfFreedom());
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

    testSparsityPatternAllRepresentations("oneLink.urdf");
    testSparsityPatternAllRepresentations("twoLinks.urdf");
    testSparsityPatternAllRepresentations("threeLinks.urdf");
    testSparsityPatternAllRepresentations("bigman.urdf");
    testSparsityPatternAllRepresentations("icub_skin_frames.urdf");



    return EXIT_SUCCESS;
}
