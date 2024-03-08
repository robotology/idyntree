// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause


#include <iDynTree/TestUtils.h>
#include <iDynTree/EigenHelpers.h>

#include <iDynTree/Model.h>
#include <iDynTree/Traversal.h>

#include <iDynTree/ForwardKinematics.h>
#include <iDynTree/Dynamics.h>
#include <iDynTree/DynamicsLinearization.h>

#include <iDynTree/JointState.h>
#include <iDynTree/LinkState.h>
#include <iDynTree/FreeFloatingState.h>
#include <iDynTree/FreeFloatingMatrices.h>

#include <iDynTree/ModelTestUtils.h>

#include <iDynTree/ModelLoader.h>

#include "testModels.h"

#include <cassert>
#include <cstdio>
#include <cstdlib>

using namespace iDynTree;

void diffArticulatedBodyAlgorithmInternalBuffers(ArticulatedBodyAlgorithmInternalBuffers & bufs, ArticulatedBodyAlgorithmInternalBuffers & perturbed)
{
    std::cout << "diffArticulatedBodyAlgorithmInternalBuffers with " <<  bufs.linksVel.getNrOfLinks() << " links " << std::endl;
    for(LinkIndex link = 0; link < bufs.linksVel.getNrOfLinks(); link++ )
    {
        std::cout << "linkVel diff \n" << toEigen(bufs.linksVel(link))-toEigen(perturbed.linksVel(link)) << std::endl;
        std::cout << "linksBiasAcceleration diff \n " << toEigen(bufs.linksBiasAcceleration(link))-toEigen(perturbed.linksBiasAcceleration(link)) << std::endl;
        std::cout << "linksAccelerations diff \n" << toEigen(bufs.linksAccelerations(link))-toEigen(perturbed.linksAccelerations(link)) << std::endl;
        std::cout << "linkBiasWrench diff \n" << toEigen(bufs.linksBiasWrench(link))-toEigen(perturbed.linksBiasWrench(link)) << std::endl;
    }
}

void checkDifferenceInBuffers(const ForwardDynamicsLinearizationInternalBuffers & bufs,
                              const ForwardDynamicsLinearizationInternalBuffers & numBufs,
                              bool verbose = true)
{
    // check the bufs related to derivative wrt to base velocity
    size_t nrOfLinks = bufs.dVb_linkBiasAcceleration.size();
    size_t nrOfDofs  = bufs.aba.D.size();

    for(size_t l = 0; l < nrOfLinks; l++ )
    {
        if( false )
        {
            std::cerr << "Check difference in buffers for link " << l << std::endl;
            std::cerr << "Norm of diff in dVb_linkBiasAcceleration : "
                      << (toEigen(bufs.dVb_linkBiasAcceleration[l])-toEigen(numBufs.dVb_linkBiasAcceleration[l])).norm() << std::endl;
            std::cerr << "Norm of diff in dVb_linksAccelerations : "
                      << (toEigen(bufs.dVb_linksAccelerations[l])-toEigen(numBufs.dVb_linksAccelerations[l])).norm() << std::endl;
            std::cerr << "Norm of diff in dVb_linkBiasWrench : "
                      << (toEigen(bufs.dVb_linkBiasWrench[l])-toEigen(numBufs.dVb_linkBiasWrench[l])).norm() << std::endl;
            std::cerr << "Norm of diff in dVb_u : "
                      << (toEigen(bufs.dVb_u[l])-toEigen(numBufs.dVb_u[l])).norm() << std::endl;

            /*
            std::cerr << "dVb_linksAccelerations symbolical: " << std::endl;
            std::cerr << toEigen(bufs.dVb_linksAccelerations[l]) << std::endl;

            std::cerr << "dVb_linksAccelerations numer: " << std::endl;
            std::cerr << toEigen(numBufs.dVb_linksAccelerations[l]) << std::endl;
            */

        }
    }

    // check the bufs related to the derivative wrt to joint velocities
    for(size_t dofDeriv = 0; dofDeriv < nrOfDofs; dofDeriv++)
    {
        if(false) std::cerr << "~~~~~~~~~ Checking buffers for dofDeriv " << dofDeriv << std::endl;
        // Check link buffers
        for(size_t l = 0; l < nrOfLinks; l++ )
        {
            if( false  )
            {
                std::cerr << "Check difference in buffers for link " << l << std::endl;
                std::cerr << "Norm of diff in linksVel : "
                      << (toEigen(bufs.dVel[dofDeriv].linksVel(l))-toEigen(numBufs.dVel[dofDeriv].linksVel(l))).norm() << std::endl;
                std::cerr << "Norm of diff in linksBiasAcceleration : "
                      << (toEigen(bufs.dVel[dofDeriv].linksBiasAcceleration(l))-toEigen(numBufs.dVel[dofDeriv].linksBiasAcceleration(l))).norm() << std::endl;
                std::cerr << "Norm of diff in linksAccelerations : "
                      << (toEigen(bufs.dVel[dofDeriv].linksAccelerations(l))-toEigen(numBufs.dVel[dofDeriv].linksAccelerations(l))).norm() << std::endl;
                std::cerr << "Norm of diff in linkBiasWrench : "
                      << (toEigen(bufs.dVel[dofDeriv].linksBiasWrench(l))-toEigen(numBufs.dVel[dofDeriv].linksBiasWrench(l))).norm() << std::endl;
                //std::cerr << "Norm of diff in pa : "
                //      << (toEigen(bufs.dVel[dofDeriv].pa(l))-toEigen(numBufs.dVel[dofDeriv].pa(l))).norm() << std::endl;

            }
        }

        // Check dof buffers
        for(size_t dof = 0; dof < nrOfDofs; dof++ )
        {
            if( verbose )
            {
                 std::cerr << "Check difference in buffers for dof " << dof << std::endl;
                 std::cerr << "Norm of diff in u : "
                           << fabs(bufs.dVel[dofDeriv].u(dof)-numBufs.dVel[dofDeriv].u(dof)) << std::endl;
            }
        }
    }

    // check the bufs related to the derivative wrt to joint positions
    for(size_t dofDeriv = 0; dofDeriv < nrOfDofs; dofDeriv++)
    {
        if(true) std::cerr << "~~~~~~~~~ Checking buffers for dofDeriv " << dofDeriv << std::endl;
        // Check link buffers
        for(size_t l = 0; l < nrOfLinks; l++ )
        {
            if( true  )
            {
                std::cerr << "Check difference in buffers for link " << l << std::endl;
                std::cerr << "Norm of diff in linksVel : "
                      << (toEigen(bufs.dPos[dofDeriv].linksVel(l))-toEigen(numBufs.dPos[dofDeriv].linksVel(l))).norm() << std::endl;
                std::cerr << "Norm of diff in linksBiasAcceleration : "
                      << (toEigen(bufs.dPos[dofDeriv].linksBiasAcceleration(l))-toEigen(numBufs.dPos[dofDeriv].linksBiasAcceleration(l))).norm() << std::endl;
                std::cerr << "Norm of diff in linksAccelerations : "
                      << (toEigen(bufs.dPos[dofDeriv].linksAccelerations(l))-toEigen(numBufs.dPos[dofDeriv].linksAccelerations(l))).norm() << std::endl;
                std::cerr << "Norm of diff in linkBiasWrench : "
                      << (toEigen(bufs.dPos[dofDeriv].linksBiasWrench(l))-toEigen(numBufs.dPos[dofDeriv].linksBiasWrench(l))).norm() << std::endl;
                //std::cerr << "Norm of diff in pa : "
                //      << (toEigen(bufs.dVel[dofDeriv].pa(l))-toEigen(numBufs.dVel[dofDeriv].pa(l))).norm() << std::endl;

            }
        }

        // Check dof buffers
        for(size_t dof = 0; dof < nrOfDofs; dof++ )
        {
            if( true )
            {
                 std::cerr << "Check difference in buffers for dof " << dof << std::endl;
                 std::cerr << "Norm of diff in u : "
                           << fabs(bufs.dPos[dofDeriv].u(dof)-numBufs.dPos[dofDeriv].u(dof)) << std::endl;
                 std::cerr << "Norm of diff in D : "
                           << fabs(bufs.dPos[dofDeriv].D(dof)-numBufs.dPos[dofDeriv].D(dof)) << std::endl;
                 std::cerr << " D "
                            << bufs.aba.D(dof) << std::endl;

            }
        }
    }

}

/**
 * Fill a single column of the linearization matrix
 *
 */
void fillLowerColumnLinearizationMatrix(FreeFloatingStateLinearization & A,
                                        FreeFloatingAcc & acc,
                                        FreeFloatingAcc & perturbedAcc,
                                        double step,
                                        size_t column)
{
    size_t nrOfDofs = acc.getNrOfDOFs();
    toEigen(A).block<6,1>(6+nrOfDofs,column) = (toEigen(perturbedAcc.baseAcc())-toEigen(acc.baseAcc()))/step;

    for(size_t dof = 0; dof < acc.getNrOfDOFs(); dof++ )
    {
        A(6+nrOfDofs+6+dof,column) = (perturbedAcc.jointAcc()(dof)-acc.jointAcc()(dof))/step;
    }
}

/**
 * Fill the buffers of the linearization procedure w.r.t the base velocity
 * using the numerical derivatives
 *
 */
void fillLinBufsWithNumericalDerivativesWrtBaseTwist(const ArticulatedBodyAlgorithmInternalBuffers & bufsLower,
                                    const ArticulatedBodyAlgorithmInternalBuffers & bufsUpper,
                                    const size_t h,
                                    const double step,
                                    ForwardDynamicsLinearizationInternalBuffers & numBufs)
{
    for(size_t l=0; l < numBufs.dVb_linkBiasAcceleration.size(); l++ )
    {
        toEigen(numBufs.dVb_linkBiasAcceleration[l]).block<6,1>(0,h) = (toEigen(bufsUpper.linksBiasAcceleration(l))-toEigen(bufsLower.linksBiasAcceleration(l)))/step;
        toEigen(numBufs.dVb_linksAccelerations[l]).block<6,1>(0,h) = (toEigen(bufsUpper.linksAccelerations(l))-toEigen(bufsLower.linksAccelerations(l)))/step;
        toEigen(numBufs.dVb_linkBiasWrench[l]).block<6,1>(0,h) = (toEigen(bufsUpper.linksBiasWrench(l))-toEigen(bufsLower.linksBiasWrench(l)))/step;
    }
}

template <class spatialVectorType, class EigenType>
void setSpatialVectorFromEigen(spatialVectorType & vec, const EigenType eigVec)
{
    toEigen(vec.getLinearVec3()) = eigVec.segment(0,3);
    toEigen(vec.getAngularVec3()) = eigVec.segment(3,3);
}

void fillLinBufsWithNumericalDerivativesWrtJointQuantity(const ArticulatedBodyAlgorithmInternalBuffers & bufsLower,
                                                         const ArticulatedBodyAlgorithmInternalBuffers & bufsUpper,
                                                         const double step,
                                                         ArticulatedBodyAlgorithmInternalBuffers & numBufs)
{
    // set links buffers
    for(size_t l=0; l < numBufs.linksBiasAcceleration.getNrOfLinks(); l++ )
    {
        setSpatialVectorFromEigen(numBufs.linksVel(l),(toEigen(bufsUpper.linksVel(l))-toEigen(bufsLower.linksVel(l)))/step);
        setSpatialVectorFromEigen(numBufs.linksBiasAcceleration(l),(toEigen(bufsUpper.linksBiasAcceleration(l))-toEigen(bufsLower.linksBiasAcceleration(l)))/step);
        setSpatialVectorFromEigen(numBufs.linksAccelerations(l),(toEigen(bufsUpper.linksAccelerations(l))-toEigen(bufsLower.linksAccelerations(l)))/step);
        setSpatialVectorFromEigen(numBufs.linksBiasWrench(l),(toEigen(bufsUpper.linksBiasWrench(l))-toEigen(bufsLower.linksBiasWrench(l)))/step);
        //setSpatialVectorFromEigen(numBufs.pa(l),(toEigen(bufsUpper.pa(l))-toEigen(bufsLower.pa(l)))/step);
    }

    // set dofs buffers
    for(size_t dof=0; dof < numBufs.u.size(); dof++ )
    {
        numBufs.u(dof) = (bufsUpper.u(dof) - bufsLower.u(dof))/step;
    }
}


/*
 * Compute the left-trivialized linearization,
 * as described in [fill with left-trivialized repot when available],
 * using numerical derivatives (performs numerical derivatives,
 * allocate
 */
bool ForwardDynamicsLinearizationNumerical(const Model& model,
                                           const Traversal& traversal,
                                           const FreeFloatingPos& robotPos,
                                           const FreeFloatingVel& robotVel,
                                           const LinkNetExternalWrenches & linkExtWrenches,
                                           const JointDOFsDoubleArray & jointTorques,
                                                 ArticulatedBodyAlgorithmInternalBuffers bufs,
                                                 ForwardDynamicsLinearizationInternalBuffers & bufsNum,
                                                 FreeFloatingAcc & robotAcc,
                                                 FreeFloatingStateLinearization & A)
{
    // First run the normal aba to get actual robot acceleration
    bool ok = true;

    ok = ok & ArticulatedBodyAlgorithm(model,traversal,robotPos,robotVel,
                                      linkExtWrenches,jointTorques,bufs,robotAcc);

    ArticulatedBodyAlgorithmInternalBuffers perturbedBufs(model);
    ArticulatedBodyAlgorithmInternalBuffers perturbedBufsLower(model);
    FreeFloatingAcc perturbedAcc(model);
    FreeFloatingAcc perturbedAccLower(model);

    A.zero();

    size_t nrOfDofs = model.getNrOfDOFs();

    // Upper half of the matrix: we use the
    // symbolic equations because the forward dynamics just returns
    // the derivatives of base velocity and of joint velocities, not
    // the derivatives of base position and of joint positions
    // the upper half (6+nDofs \times 2*(6+nDofs)) of the linearization
    // matrix is
    //
    //   V_b \times  0     I    0
    //   0           0     0    I
    //
    Matrix6x6 VbCross = robotVel.baseVel().asCrossProductMatrix();
    toEigen(A).block(0,0,6,6) = toEigen(VbCross);

    toEigen(A).block(0,6+nrOfDofs,6,6).setIdentity();

    toEigen(A).block(6,6+nrOfDofs+6,nrOfDofs,nrOfDofs).setIdentity();


    // Lower half of the matrix: computed by numerical differentiation
    // of the forward dynamics (computed using the Articulated Body Algorithm)
    // Dealing with the easy part for now: derivative of acceleration
    // with respect to the base twist, joint position and accelerations
    FreeFloatingPos perturbedRobotPos(model);
    perturbedRobotPos.worldBasePos() = robotPos.worldBasePos();
    toEigen(perturbedRobotPos.jointPos()) = toEigen(robotPos.jointPos());

    FreeFloatingPos perturbedRobotPosLower(model);
    perturbedRobotPosLower.worldBasePos() = robotPos.worldBasePos();
    toEigen(perturbedRobotPosLower.jointPos()) = toEigen(robotPos.jointPos());

    // Copy its from the input
    FreeFloatingVel perturbedRobotVel(model);
    perturbedRobotVel.baseVel() = robotVel.baseVel();
    toEigen(perturbedRobotVel.jointVel()) = toEigen(robotVel.jointVel());

    FreeFloatingVel perturbedRobotVelLower(model);
    perturbedRobotVelLower.baseVel() = robotVel.baseVel();
    toEigen(perturbedRobotVelLower.jointVel()) = toEigen(robotVel.jointVel());

    // Step of the numerical derivatives
    double step = 1e-4;

    // Don't doing derivative with respect to the base position for now

    // Joint position derivative
    for(size_t h = 0; h < model.getNrOfDOFs(); h++)
    {
        // Add perturbation to the h element of joint position
        perturbedRobotPos.jointPos()(h) = robotPos.jointPos()(h) + step/2;
        perturbedRobotPosLower.jointPos()(h) = robotPos.jointPos()(h) - step/2;

        // We run the ABA with the perturbed input
        ArticulatedBodyAlgorithm(model,traversal,perturbedRobotPos,perturbedRobotVel,
                                 linkExtWrenches,jointTorques,perturbedBufs,perturbedAcc);

        ArticulatedBodyAlgorithm(model,traversal,perturbedRobotPosLower,perturbedRobotVelLower,
                                 linkExtWrenches,jointTorques,perturbedBufsLower,perturbedAccLower);

        // Fill the matrix column
        fillLowerColumnLinearizationMatrix(A,perturbedAccLower,perturbedAcc,step,h+6);

        // Restored the old value
        perturbedRobotPos.jointPos()(h) = robotPos.jointPos()(h);
        perturbedRobotPosLower.jointPos()(h) = robotPos.jointPos()(h);
    }

    // Base vel derivative
    for(size_t h = 0; h < 6; h++)
    {
        // Add perturbation to the h element of the base velocity
        perturbedRobotVel.baseVel()(h) = robotVel.baseVel()(h) + step/2;
        perturbedRobotVelLower.baseVel()(h) = robotVel.baseVel()(h) - step/2;

        // We run the ABA with the perturbed input
        ArticulatedBodyAlgorithm(model,traversal,perturbedRobotPos,perturbedRobotVel,
                                 linkExtWrenches,jointTorques,perturbedBufs,perturbedAcc);

        ArticulatedBodyAlgorithm(model,traversal,perturbedRobotPosLower,perturbedRobotVelLower,
                                 linkExtWrenches,jointTorques,perturbedBufsLower,perturbedAccLower);

        // We compute the buffers values using numerical diff
        fillLinBufsWithNumericalDerivativesWrtBaseTwist(perturbedBufsLower,perturbedBufs,h,step,bufsNum);

        // Fill the matrix column
        fillLowerColumnLinearizationMatrix(A,perturbedAccLower,perturbedAcc,step,h+6+model.getNrOfDOFs());

        // Restored the old value
        perturbedRobotVel.baseVel()(h) = robotVel.baseVel()(h);
        perturbedRobotVelLower.baseVel()(h) = robotVel.baseVel()(h);
    }

    // Joint vel derivative
    for(size_t h = 0; h < model.getNrOfDOFs(); h++)
    {
        // Add perturbation to the h element of joint velocity
        perturbedRobotVel.jointVel()(h) = robotVel.jointVel()(h) + step/2;
        perturbedRobotVelLower.jointVel()(h) = robotVel.jointVel()(h) - step/2;

        // We run the ABA with the perturbed input
        ArticulatedBodyAlgorithm(model,traversal,perturbedRobotPos,perturbedRobotVel,
                                 linkExtWrenches,jointTorques,perturbedBufs,perturbedAcc);

        ArticulatedBodyAlgorithm(model,traversal,perturbedRobotPosLower,perturbedRobotVelLower,
                                 linkExtWrenches,jointTorques,perturbedBufsLower,perturbedAccLower);

        // Fill the matrix column
        fillLowerColumnLinearizationMatrix(A,perturbedAccLower,perturbedAcc,step,h+6+6+model.getNrOfDOFs());

        // we compute also the buffers values using the numerical differences
        fillLinBufsWithNumericalDerivativesWrtJointQuantity(perturbedBufsLower,perturbedBufs,step,bufsNum.dVel[h]);

        // Restored the old value
        perturbedRobotVel.jointVel()(h) = robotVel.jointVel()(h);
        perturbedRobotVelLower.jointVel()(h) = robotVel.jointVel()(h);
    }

    return ok;
}


/**
 * Check that the numerical linearization of the ABA and
 * its symbolical derivation match.
 */
void checkABAandABALinearizationAreConsistent(const Model & model,
                                              const Traversal & traversal)
{
    // Input data
     // Allocate input for both algorithms : robot position, velocity
    // and link external wrenches
    LinkNetExternalWrenches linkExtWrenches(model);
    FreeFloatingPos   robotPos(model);
    FreeFloatingVel   robotVel(model);
    FreeFloatingAcc   robotAcc(model);

    // Input for direct dynamics algorithms
    // and output for inverse dynamics : joint torques
    JointDOFsDoubleArray jntTorques(model);

    // Fill the input to forward dynamics with random data
    robotPos.worldBasePos() = iDynTree::Transform::Identity();
    //robotPos.worldBasePos() = getRandomTransform();

    //robotVel.baseVel() = getRandomTwist();
    robotVel.baseVel().zero();
    //robotVel.baseVel()(1) = 4.0;
    //robotVel.baseVel()(4) = 1.0;

    //getRandomVector(robotPos.jointPos());
    robotPos.jointPos().zero();
    //robotPos.jointPos()(0) = 3.14;

    //getRandomVector(robotVel.jointVel());
    robotVel.jointVel().zero();


    for(unsigned int link=0; link < model.getNrOfLinks(); link++ )
    {
        linkExtWrenches(link) =  getRandomWrench();
    }
    getRandomVector(jntTorques);

    // Create buffers for the algorithms
    ArticulatedBodyAlgorithmInternalBuffers abaBufs(model);
    ForwardDynamicsLinearizationInternalBuffers bufs(model);
    // Using numerical differentiation we also compute the numerical
    // derivatives of the internal buffes, to simplify debugging
    ForwardDynamicsLinearizationInternalBuffers bufsNumerical(model);

    FreeFloatingStateLinearization A(model);
    FreeFloatingStateLinearization Anumerical(model);

    // Compute matrix with numerical derivatives
    ForwardDynamicsLinearizationNumerical(model,traversal,robotPos,robotVel,
                                          linkExtWrenches,jntTorques,abaBufs,bufsNumerical,
                                          robotAcc,Anumerical);

    ForwardDynamicsLinearization(model,traversal,robotPos,robotVel,
                                 linkExtWrenches,jntTorques,bufs,
                                 robotAcc,A);

    Eigen::IOFormat HeavyFmt(4, 0, ", ", ";\n", "[", "]", "[", "]");



    std::cerr << " A : " << std::endl;
    std::cerr  << toEigen(A).format(HeavyFmt) << std::endl;

    std::cerr << " A (computed with numerical derivatives): " << std::endl;
    std::cerr << toEigen(Anumerical).format(HeavyFmt) << std::endl;

    std::cerr << "Diff " << std::endl;
    std::cerr << (toEigen(A)-toEigen(Anumerical)).format(HeavyFmt) << std::endl;

    // Check also buffers
    checkDifferenceInBuffers(bufs,bufsNumerical,false);

    ASSERT_EQUAL_MATRIX_TOL(A,Anumerical,1e-2);

    return;
}

/**
 * overloaded version that uses the default traversal.
 */
void checkABAandABALinearizationAreConsistent(const Model & model)
{
    Traversal traversal;
    model.computeFullTreeTraversal(traversal);

    checkABAandABALinearizationAreConsistent(model,traversal);
}


int main()
{
    // First test some models used for debug
    Model doubleBodyModel;

    RotationalInertia rotInertia = RotationalInertia::Zero();
    rotInertia(0,0) = rotInertia(1,1) = rotInertia(2,2) = 1.0;

    Position com = Position::Zero();
    com(0) = 0.0;
    com(1) = 0.0;
    com(2) = 0.0;

    SpatialInertia singleBodyInertia;
    singleBodyInertia.fromRotationalInertiaWrtCenterOfMass(1.0,com,rotInertia);
    Link pointMassLink;
    pointMassLink.setInertia(singleBodyInertia);
    doubleBodyModel.addLink("link1",pointMassLink);
    doubleBodyModel.addLink("link2",pointMassLink);

    iDynTree::Axis axis(Direction(0,0,1),Position::Zero());

    RevoluteJoint* p_joint = new RevoluteJoint();
    p_joint->setAttachedLinks(0,1);
    p_joint->setRestTransform(iDynTree::Transform(Rotation::RPY(0,0,0),Position(0.0,0.0,0.0)));
    p_joint->setAxis(axis, 1);
    //IJoint* p_joint = new FixedJoint(0,1,
    //                            iDynTree::Transform(Rotation::RPY(0,0,0),Position(0.0,0.0,0.0)));

    p_joint->setAttachedLinks(0,1);

    doubleBodyModel.addJoint("joint1",p_joint);

    delete p_joint;

    std::cerr << "Checking DynamicsLinearization test on a point mass model" << std::endl;
    checkABAandABALinearizationAreConsistent(doubleBodyModel);

    // Then test random generated chains
    for(unsigned int joints =0; joints < 20; joints++)
    {
        std::cerr << "Checking DynamicsLinearization test on random chain with " << joints << " joints " <<  std::endl;
        Model model = getRandomChain(joints);
        checkABAandABALinearizationAreConsistent(model);
    }

    // Then test random generated trees
    for(unsigned int joints =0; joints < 20; joints++)
    {
        std::cerr << "Checking DynamicsLinearization test on on random tree with " << joints  << "joints " << std::endl;
        Model model = getRandomModel(joints);
        checkABAandABALinearizationAreConsistent(model);
    }

    // Then test URDF models usually used for testing
    for(unsigned int mdl = 0; mdl < IDYNTREE_TESTS_URDFS_NR; mdl++ )
    {
        std::string urdfFileName = getAbsModelPath(std::string(IDYNTREE_TESTS_URDFS[mdl]));
        std::cerr << "Checking DynamicsLinearization test on " << urdfFileName << std::endl;

        ModelLoader loader;
        loader.loadModelFromFile(urdfFileName);
        Model model = loader.model();
        checkABAandABALinearizationAreConsistent(model);
    }
}
