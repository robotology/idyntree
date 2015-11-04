/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "testModels.h"

// KDL related includes
#include <kdl_codyco/position_loops.hpp>
#include <kdl_codyco/rnea_loops.hpp>
#include <kdl_codyco/undirectedtree.hpp>
#include <kdl_codyco/KDLConversions.h>

#include <iDynTree/ModelIO/impl/urdf_import.hpp>

// iDynTree includes
#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/FixedJoint.h>
#include <iDynTree/Model/RevoluteJoint.h>

#include <iDynTree/Model/ForwardKinematics.h>
#include <iDynTree/Model/LinkState.h>
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Model/FreeFloatingState.h>

#include <iDynTree/ModelIO/URDFModelImport.h>

#include <iDynTree/Core/TestUtils.h>

#include <iDynTree/Core/EigenHelpers.h>

using namespace iDynTree;

void testFwdKinConsistency(std::string modelFilePath)
{
    // Load iDynTree model and compute a default traversal
    iDynTree::Model model;
    modelFromURDF(modelFilePath,model);
    iDynTree::Traversal traversal;
    model.computeFullTreeTraversal(traversal);

    // Load KDL tree
    KDL::Tree tree;
    treeFromUrdfFile(modelFilePath,tree);
    KDL::CoDyCo::UndirectedTree undirected_tree(tree);
    KDL::CoDyCo::Traversal kdl_traversal;
    undirected_tree.compute_traversal(kdl_traversal);

    // A KDL::Tree only supports 0 and 1 DOFs joints, and
    // KDL::Tree::getNrOfJoints does not count the 0 dof joints, so
    // it is actually equivalent to iDynTree::Model::getNrOfDOFs
    ASSERT_EQUAL_DOUBLE(tree.getNrOfJoints(),model.getNrOfDOFs());

    std::cout << "KDL Links :" <<  undirected_tree.getNrOfLinks() << std::endl;
    std::cout << "iDynTree Links :" <<  model.getNrOfLinks() << std::endl;

    // Input : joint positions and base position with respect to world
    iDynTree::FreeFloatingPos baseJntPos(model);
    iDynTree::FreeFloatingPosVelAcc baseJntPosVelAcc(model);

    baseJntPos.worldBasePos() =  getRandomTransform();
    baseJntPosVelAcc.basePosVelAcc().pos() =  baseJntPos.worldBasePos();
    baseJntPosVelAcc.basePosVelAcc().vel() =  getRandomTwist();
    baseJntPosVelAcc.basePosVelAcc().acc() = getRandomTwist();

    for(int jnt=0; jnt < baseJntPos.getNrOfPosCoords(); jnt++)
    {
        baseJntPos.jointPos()(jnt) = getRandomDouble();
        baseJntPosVelAcc.jointPos()(jnt) = baseJntPos.jointPos()(jnt);
        baseJntPosVelAcc.jointVel()(jnt) = getRandomDouble();
        baseJntPosVelAcc.jointAcc()(jnt) = getRandomDouble();
    }


    // Build a map between KDL joints and iDynTree joints because we are not sure
    // that the joint indeces will match
    std::vector<int> kdl2idyntree_joints;
    kdl2idyntree_joints.resize(undirected_tree.getNrOfDOFs());

    for(unsigned int dofIndex=0; dofIndex < undirected_tree.getNrOfDOFs(); dofIndex++)
    {
        std::string dofName = undirected_tree.getJunction(dofIndex)->getName();
        int idyntreeJointIndex = model.getJointIndex(dofName);
        kdl2idyntree_joints[dofIndex] = idyntreeJointIndex;
    }


    KDL::JntArray jntKDL(undirected_tree.getNrOfDOFs());
    KDL::JntArray jntVelKDL(undirected_tree.getNrOfDOFs());
    KDL::JntArray jntAccKDL(undirected_tree.getNrOfDOFs());

    KDL::Frame  worldBaseKDL;
    KDL::Twist  baseVelKDL;
    KDL::Twist  baseAccKDL;

    ToKDL(baseJntPosVelAcc,worldBaseKDL,jntKDL,
                           baseVelKDL,jntVelKDL,
                           baseAccKDL,jntAccKDL,kdl2idyntree_joints);

    // Output : link (for iDynTree) or frame positions with respect to world
    std::vector<KDL::Frame> framePositions(undirected_tree.getNrOfLinks());

    iDynTree::LinkPositions linkPositions(model);


    // Build a map between KDL links and iDynTree links because we are not sure
    // that the link indeces will match
    std::vector<int> idynTree2KDL_links;
    idynTree2KDL_links.resize(model.getNrOfLinks());

    for(unsigned int linkIndex=0; linkIndex < model.getNrOfLinks(); linkIndex++)
    {
        std::string linkName = model.getLinkName(linkIndex);
        int kdlLinkIndex = undirected_tree.getLink(linkName)->getLinkIndex();
        idynTree2KDL_links[linkIndex] = kdlLinkIndex;
    }

    // Compute position forward kinematics with old KDL code and with the new iDynTree code
    KDL::CoDyCo::getFramesLoop(undirected_tree,
                               jntKDL,
                               kdl_traversal,
                               framePositions,
                               worldBaseKDL);

    ForwardPositionKinematics(model,traversal,baseJntPos,linkPositions);

    // Check results
    for(unsigned int travEl = 0; travEl  < traversal.getNrOfVisitedLinks(); travEl++)
    {
        LinkIndex linkIndex = traversal.getLink(travEl)->getIndex();
        ASSERT_EQUAL_TRANSFORM_TOL(linkPositions.linkPos(linkIndex).pos(),ToiDynTree(framePositions[idynTree2KDL_links[linkIndex]]),1e-1);
    }

    // Compution velocity & acceleration kinematics
    std::vector<KDL::Twist> kdlLinkVel(undirected_tree.getNrOfLinks());
    std::vector<KDL::Twist> kdlLinkAcc(undirected_tree.getNrOfLinks());
    KDL::CoDyCo::rneaKinematicLoop(undirected_tree,jntKDL,jntVelKDL,jntAccKDL,kdl_traversal,
                                                   baseVelKDL,baseAccKDL,kdlLinkVel,kdlLinkAcc);

    LinkVelAccArray linksVelAcc(model);

    ForwardVelAccKinematics(model,traversal,baseJntPosVelAcc,linksVelAcc);

    // Check results
    for(unsigned int travEl = 0; travEl  < traversal.getNrOfVisitedLinks(); travEl++)
    {
        LinkIndex linkIndex = traversal.getLink(travEl)->getIndex();

        std::cout << " link " <<  model.getLinkName(linkIndex) << std::endl;
        std::cout << linksVelAcc.linkVelAcc(linkIndex).acc().asVector().toString() << std::endl;
        std::cout << ToiDynTree(kdlLinkAcc[idynTree2KDL_links[linkIndex]]).asVector().toString() << std::endl;


        ASSERT_EQUAL_VECTOR(linksVelAcc.linkVelAcc(linkIndex).vel().asVector(),
                            ToiDynTree(kdlLinkVel[idynTree2KDL_links[linkIndex]]).asVector());
        ASSERT_EQUAL_VECTOR(linksVelAcc.linkVelAcc(linkIndex).acc().asVector(),
                            ToiDynTree(kdlLinkAcc[idynTree2KDL_links[linkIndex]]).asVector());
    }

    // Compute position, velocity and acceleration
    LinkPosVelAccArray linksPosVelAcc(model);

    ForwardPosVelAccKinematics(model,traversal,baseJntPosVelAcc,linksPosVelAcc);

    for(unsigned int travEl = 0; travEl  < traversal.getNrOfVisitedLinks(); travEl++)
    {
        LinkIndex linkIndex = traversal.getLink(travEl)->getIndex();
        ASSERT_EQUAL_TRANSFORM(linkPositions.linkPos(linkIndex).pos(),
                               linksPosVelAcc.linkPosVelAcc(linkIndex).pos());
        ASSERT_EQUAL_VECTOR(linksVelAcc.linkVelAcc(linkIndex).vel().asVector(),
                            linksPosVelAcc.linkPosVelAcc(linkIndex).vel().asVector());
        ASSERT_EQUAL_VECTOR(linksVelAcc.linkVelAcc(linkIndex).acc().asVector(),
                            linksPosVelAcc.linkPosVelAcc(linkIndex).acc().asVector());
    }

    return;
}

int main()
{
    for(unsigned int mdl = 0; mdl < IDYNTREE_TESTS_URDFS_NR; mdl++ )
    {
        std::string urdfFileName = getAbsModelPath(std::string(IDYNTREE_TESTS_URDFS[mdl]));
        testFwdKinConsistency(urdfFileName);
    }

    return EXIT_SUCCESS;
}
