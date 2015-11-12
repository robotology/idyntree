/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


#include <iDynTree/Core/TestUtils.h>

#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/Traversal.h>

#include <iDynTree/ModelIO/URDFModelImport.h>

#include <iDynTree/Model/ForwardKinematics.h>
#include <iDynTree/Model/Dynamics.h>

#include <iDynTree/Model/JointState.h>
#include <iDynTree/Model/LinkState.h>
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/Model/FreeFloatingMassMatrix.h>

#include "testModels.h"

#include <cassert>
#include <cstdio>
#include <cstdlib>

using namespace iDynTree;

void checkInverseAndForwardDynamicsAreIdempotent(const Model & model,
                                                 const Traversal & traversal)
{
    // Allocate input for both algorithms : robot position, velocity
    // and link external wrenches
    LinkExternalWrenches linkExtWrenches(model);
    FreeFloatingPosVel   robotPosVel(model);

    // Input for direct dynamics algorithms
    // and output for inverse dynamics : joint torques
    JointDoubleArray ABA_jntTorques(model);

    // Fill the input to forward dynamics with random data
    robotPosVel.basePosVel().pos() = getRandomTransform();
    robotPosVel.basePosVel().vel() = getRandomTwist();
    getRandomVector(robotPosVel.jointPos());
    getRandomVector(robotPosVel.jointVel());
    for(unsigned int link=0; link < model.getNrOfLinks(); link++ )
    {
        linkExtWrenches(link) = getRandomWrench();
    }
    getRandomVector(ABA_jntTorques);

    // Output for direct dynamics algorithms
    // and input for inverse dynamics : robot acceleration
    FreeFloatingAcc ABA_robotAcc(model);

    // Allocate temporary data structures for ABA
    iDynTree::DOFSpatialMotionArray ABA_S(model);
    iDynTree::DOFSpatialForceArray ABA_U(model);
    iDynTree::JointDoubleArray ABA_D(model);
    JointDoubleArray ABA_u(model);
    iDynTree::LinkVelArray ABA_linksVel(model);
    iDynTree::LinkAccArray ABA_linksBiasAcceleration(model);
    ASSERT_EQUAL_DOUBLE(ABA_linksBiasAcceleration.getNrOfLinks(),model.getNrOfLinks());
    LinkAccArray ABA_linksAcceleration(model);
    ASSERT_EQUAL_DOUBLE(ABA_linksBiasAcceleration.getNrOfLinks(),model.getNrOfLinks());
    LinkArticulatedBodyInertias linkABIs(model);
    LinkWrenches linksBiasWrench(model);

    // Run ABA
    ArticulatedBodyAlgorithm(model,
                             traversal,
                             robotPosVel,
                             linkExtWrenches,
                             ABA_jntTorques,
                             ABA_S,
                             ABA_U,
                             ABA_D,
                             ABA_u,
                             ABA_linksVel,
                             ABA_linksBiasAcceleration,
                             ABA_linksAcceleration,
                             linkABIs,
                             linksBiasWrench,
                             ABA_robotAcc);

    // Allocate temporary data structure for RNEA
    LinkVelAccArray RNEA_linksVelAccs(model);
    LinkInternalWrenches RNEA_linkIntWrenches(model);
    FreeFloatingGeneralizedTorques RNEA_baseForceAndJointTorques(model);
    FreeFloatingPosVelAcc   RNEA_robotPosVelAcc(model);

    // Fill input for RNEA
    RNEA_robotPosVelAcc.basePosVelAcc().pos() = robotPosVel.basePosVel().pos();
    RNEA_robotPosVelAcc.basePosVelAcc().vel() = robotPosVel.basePosVel().vel();
    RNEA_robotPosVelAcc.basePosVelAcc().acc() = ABA_robotAcc.baseAcc();
    for(unsigned int dof=0; dof < model.getNrOfDOFs(); dof++)
    {
        RNEA_robotPosVelAcc.jointPos()(dof) = robotPosVel.jointPos()(dof);
        RNEA_robotPosVelAcc.jointVel()(dof) = robotPosVel.jointVel()(dof);
        RNEA_robotPosVelAcc.jointAcc()(dof) = ABA_robotAcc.jointAcc()(dof);
    }

    // Run RNEA
    ForwardVelAccKinematics(model,
                            traversal,
                            RNEA_robotPosVelAcc,
                            RNEA_linksVelAccs);
    RNEADynamicPhase(model,
                     traversal,
                     RNEA_robotPosVelAcc,
                     RNEA_linksVelAccs,
                     linkExtWrenches,
                     RNEA_linkIntWrenches,
                     RNEA_baseForceAndJointTorques);

    // Check output of RNEA : given that accelerations
    // and external forces are consistent, the resulting
    // base wrench should be zero, while the joint torque
    // should be equal to the one given in input to the ABA
    Vector6 zeroVec; zeroVec.zero();
    std::cout << "Check base wrench" << std::endl;
    ASSERT_EQUAL_VECTOR_TOL(RNEA_baseForceAndJointTorques.baseWrench().asVector(),
                        zeroVec,1e-7);
    std::cout << "Check joint torques" << std::endl;
    ASSERT_EQUAL_VECTOR_TOL(RNEA_baseForceAndJointTorques.jointTorques(),ABA_jntTorques,1e-9);
}

int main()
{
    for(unsigned int mdl = 0; mdl < IDYNTREE_TESTS_URDFS_NR; mdl++ )
    {
        std::string urdfFileName = getAbsModelPath(std::string(IDYNTREE_TESTS_URDFS[mdl]));
        std::cout << "Checking dynamics test on " << urdfFileName << std::endl;
        Model model;
        bool ok = modelFromURDF(urdfFileName,model);
        assert(ok);
        Traversal traversal;
        ok = model.computeFullTreeTraversal(traversal);
        assert(ok);
        checkInverseAndForwardDynamicsAreIdempotent(model,traversal);
    }
}
