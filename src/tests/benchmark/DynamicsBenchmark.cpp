// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "testModels.h"

// KDL related includes
#include <kdl_codyco/position_loops.hpp>
#include <kdl_codyco/rnea_loops.hpp>
#include <kdl_codyco/crba_loops.hpp>
#include <kdl_codyco/undirectedtree.hpp>
#include <kdl_codyco/KDLConversions.h>

#include <iDynTree/impl/urdf_import.hpp>

// iDynTree includes
#include <iDynTree/Model.h>
#include <iDynTree/FixedJoint.h>
#include <iDynTree/RevoluteJoint.h>

#include <iDynTree/ForwardKinematics.h>
#include <iDynTree/Dynamics.h>

#include <iDynTree/LinkState.h>
#include <iDynTree/Traversal.h>
#include <iDynTree/FreeFloatingState.h>
#include <iDynTree/FreeFloatingMatrices.h>

#include <iDynTree/ModelLoader.h>

#include <iDynTree/TestUtils.h>

#include <iDynTree/EigenHelpers.h>

using namespace iDynTree;

#include <ctime>

/**
 * Return the current time in seconds, with respect
 * to an arbitrary point in time.
 */
inline double clockInSec()
{
    clock_t ret = clock();
    return ((double)ret)/((double)CLOCKS_PER_SEC);
}

void dynamicsBenchmark(std::string modelFilePath, unsigned int nrOfTrials)
{
    std::cout << "Benchmarking dynamics algorithms for " << modelFilePath << std::endl;

    // initialization variables
    ModelLoader loader;
    loader.loadModelFromFile(modelFilePath);
    iDynTree::Model model = loader.model();
    iDynTree::Traversal traversal;
    model.computeFullTreeTraversal(traversal);

    // Load KDL tree
    KDL::Tree tree;
    treeFromUrdfFile(modelFilePath,tree);
    KDL::CoDyCo::UndirectedTree undirected_tree(tree);
    KDL::CoDyCo::Traversal kdl_traversal;
    undirected_tree.compute_traversal(kdl_traversal);

    // Input : iDynTree data structures
    iDynTree::FreeFloatingPos baseJntPos(model);
    iDynTree::FreeFloatingVel baseJntVel(model);
    iDynTree::FreeFloatingAcc baseJntAcc(model);
    iDynTree::LinkVelArray linksVels(model);
    iDynTree::LinkAccArray linksAcc(model);
    iDynTree::LinkPositions linkPositions(model);

    baseJntPos.worldBasePos() =  getRandomTransform();
    baseJntVel.baseVel() = getRandomTwist();
    baseJntAcc.baseAcc() =  getRandomTwist();

    for(unsigned int jnt=0; jnt < baseJntPos.getNrOfPosCoords(); jnt++)
    {
        baseJntPos.jointPos()(jnt) = getRandomDouble();
        baseJntVel.jointVel()(jnt) = getRandomDouble();
        baseJntAcc.jointAcc()(jnt) = getRandomDouble();
    }

    KDL::JntArray jntKDL(undirected_tree.getNrOfDOFs());
    KDL::JntArray jntVelKDL(undirected_tree.getNrOfDOFs());
    KDL::JntArray jntAccKDL(undirected_tree.getNrOfDOFs());

    KDL::Frame  worldBaseKDL;
    KDL::Twist  baseVelKDL;
    KDL::Twist  baseAccKDL;

    std::vector<KDL::Twist> kdlLinkVel(undirected_tree.getNrOfLinks());
    std::vector<KDL::Twist> kdlLinkAcc(undirected_tree.getNrOfLinks());

    LinkNetExternalWrenches fExt(model);
    LinkInternalWrenches f(model);
    FreeFloatingGeneralizedTorques baseWrenchJointTorques(model);
    KDL::JntArray jntTorques(model.getNrOfDOFs());
    KDL::Wrench   baseWrench;

    std::vector<KDL::Wrench> fExtKDL(undirected_tree.getNrOfLinks());
    std::vector<KDL::Wrench> fKDL(undirected_tree.getNrOfLinks());

    iDynTree::FreeFloatingMassMatrix massMatrix(model);
    iDynTree::LinkInertias crbis(model);

    KDL::CoDyCo::FloatingJntSpaceInertiaMatrix massMatrixKDL(undirected_tree.getNrOfDOFs()+6);
    std::vector<KDL::RigidBodyInertia> Ic(undirected_tree.getNrOfLinks());

    struct benchTime
    {
        double totalTimeRNEA;
        double totalTimeCRBA;
    };

    benchTime KDLtimes;
    KDLtimes.totalTimeCRBA = 0.0;
    KDLtimes.totalTimeRNEA = 0.0;
    benchTime iDynTreeTimes;
    iDynTreeTimes.totalTimeCRBA = 0.0;
    iDynTreeTimes.totalTimeRNEA = 0.0;

    double tic,toc;
    for(unsigned int trial=0; trial < nrOfTrials; trial++ )
    {
        tic = clockInSec();
        KDL::CoDyCo::rneaKinematicLoop(undirected_tree,jntKDL,jntVelKDL,jntAccKDL,kdl_traversal,
                                                   baseVelKDL,baseAccKDL,kdlLinkVel,kdlLinkAcc);
        KDL::CoDyCo::rneaDynamicLoop(undirected_tree,jntKDL,kdl_traversal,
                                 kdlLinkVel,kdlLinkAcc,
                                 fExtKDL,fKDL,jntTorques,baseWrench);
        toc = clockInSec();
        KDLtimes.totalTimeRNEA += (toc-tic);

        tic = clockInSec();
        ForwardVelAccKinematics(model,traversal,baseJntPos,baseJntVel,baseJntAcc,linksVels,linksAcc);
        RNEADynamicPhase(model,traversal,
                               baseJntPos.jointPos(),
                               linksVels,linksAcc,fExt,f,baseWrenchJointTorques);
        toc = clockInSec();
        iDynTreeTimes.totalTimeRNEA += (toc-tic);

        tic = clockInSec();
        CompositeRigidBodyAlgorithm(model,traversal,baseJntPos.jointPos(),crbis,massMatrix);
        toc = clockInSec();
        iDynTreeTimes.totalTimeCRBA += (toc-tic);

        tic = clockInSec();
        KDL::CoDyCo::crba_floating_base_loop(undirected_tree,kdl_traversal,jntKDL,Ic,massMatrixKDL);
        toc = clockInSec();
        KDLtimes.totalTimeCRBA += (toc-tic);
    }

    std::cout << "KDL-based implementation : " << std::endl;
    std::cout << "\tRNEA average time " << (KDLtimes.totalTimeRNEA/nrOfTrials)*1e6 << " microseconds" << std::endl;
    std::cout << "\tCRBA average time " << (KDLtimes.totalTimeCRBA/nrOfTrials)*1e6 << " microseconds" << std::endl;

    std::cout << "iDynTree-based implementation : " << std::endl;
    std::cout << "\tRNEA average time " << (iDynTreeTimes.totalTimeRNEA/nrOfTrials)*1e6 << " microseconds" << std::endl;
    std::cout << "\tCRBA average time " << (iDynTreeTimes.totalTimeCRBA/nrOfTrials)*1e6 << " microseconds" << std::endl;

    std::cout << "iDynTree/KDL ratio : " << std::endl;
    std::cout << "\tRNEA ratio " << iDynTreeTimes.totalTimeRNEA/KDLtimes.totalTimeRNEA << std::endl;
    std::cout << "\tCRBA ratio " << iDynTreeTimes.totalTimeCRBA/KDLtimes.totalTimeCRBA << std::endl;


    return;
}


int main()
{
    std::cout << "Dynamics benchmark, iDynTree built in " << IDYNTREE_CMAKE_BUILD_TYPE << " mode " << std::endl;
    int nrOfTrials = 1000;
    for(unsigned int mdl = 0; mdl < IDYNTREE_TESTS_URDFS_NR; mdl++ )
    {
        std::string urdfFileName = getAbsModelPath(std::string(IDYNTREE_TESTS_URDFS[mdl]));
        dynamicsBenchmark(urdfFileName,nrOfTrials);
    }

    return EXIT_SUCCESS;
}
