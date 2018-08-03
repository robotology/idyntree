/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */


#include <iDynTree/Core/TestUtils.h>

#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/Traversal.h>

#include <iDynTree/ModelIO/ModelLoader.h>

#include <iDynTree/Model/ForwardKinematics.h>
#include <iDynTree/Model/Dynamics.h>

#include <iDynTree/Model/JointState.h>
#include <iDynTree/Model/LinkState.h>
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/Model/FreeFloatingMatrices.h>

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
    LinkNetExternalWrenches linkExtWrenches(model);
    FreeFloatingPos   robotPos(model);
    FreeFloatingVel   robotVel(model);

    // Input for direct dynamics algorithms
    // and output for inverse dynamics : joint torques
    JointDOFsDoubleArray ABA_jntTorques(model);

    // Fill the input to forward dynamics with random data
    robotPos.worldBasePos() = getRandomTransform();
    robotVel.baseVel() = getRandomTwist();
    getRandomVector(robotPos.jointPos());
    getRandomVector(robotVel.jointVel());
    for(unsigned int link=0; link < model.getNrOfLinks(); link++ )
    {
        linkExtWrenches(link) = getRandomWrench();
    }
    getRandomVector(ABA_jntTorques);

    // Output for direct dynamics algorithms
    // and input for inverse dynamics : robot acceleration
    FreeFloatingAcc ABA_robotAcc(model);

    // Allocate temporary data structures for ABA
    iDynTree::ArticulatedBodyAlgorithmInternalBuffers ABAbufs(model);
    ASSERT_EQUAL_DOUBLE(ABAbufs.linksBiasAcceleration.getNrOfLinks(),model.getNrOfLinks());


    // Run ABA
    ArticulatedBodyAlgorithm(model,
                             traversal,
                             robotPos,
                             robotVel,
                             linkExtWrenches,
                             ABA_jntTorques,
                             ABAbufs,
                             ABA_robotAcc);

    // Allocate temporary data structure for RNEA
    LinkVelArray RNEA_linksVel(model);
    LinkAccArray RNEA_linksAcc(model);
    LinkInternalWrenches RNEA_linkIntWrenches(model);
    FreeFloatingGeneralizedTorques RNEA_baseForceAndJointTorques(model);

    // Run RNEA
    ForwardVelAccKinematics(model,
                            traversal,
                            robotPos,
                            robotVel,
                            ABA_robotAcc,
                            RNEA_linksVel,
                            RNEA_linksAcc);

    RNEADynamicPhase(model,
                     traversal,
                     robotPos.jointPos(),
                     RNEA_linksVel,
                     RNEA_linksAcc,
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
                        zeroVec,1e-6);
    std::cout << "Check joint torques" << std::endl;
    ASSERT_EQUAL_VECTOR_TOL(RNEA_baseForceAndJointTorques.jointTorques(),ABA_jntTorques,1e-07);
}

int main()
{
    for(unsigned int mdl = 0; mdl < IDYNTREE_TESTS_URDFS_NR; mdl++ )
    {
        std::string urdfFileName = getAbsModelPath(std::string(IDYNTREE_TESTS_URDFS[mdl]));
        std::cout << "Checking dynamics test on " << urdfFileName << std::endl;
        ModelLoader loader;
        bool ok = loader.loadModelFromFile(urdfFileName);
        assert(ok);
        Model model = loader.model();
        Traversal traversal;
        ok = model.computeFullTreeTraversal(traversal);
        assert(ok);
        checkInverseAndForwardDynamicsAreIdempotent(model,traversal);
    }
}
