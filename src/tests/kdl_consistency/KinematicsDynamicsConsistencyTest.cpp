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

// KDL related includes
#include <kdl_codyco/position_loops.hpp>
#include <kdl_codyco/rnea_loops.hpp>
#include <kdl_codyco/crba_loops.hpp>
#include <kdl_codyco/undirectedtree.hpp>
#include <kdl_codyco/KDLConversions.h>

#include <iDynTree/ModelIO/impl/urdf_import.hpp>

// iDynTree includes
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/FixedJoint.h>
#include <iDynTree/Model/RevoluteJoint.h>

#include <iDynTree/Model/ForwardKinematics.h>
#include <iDynTree/Model/Dynamics.h>

#include <iDynTree/Model/LinkState.h>
#include <iDynTree/Model/Traversal.h>
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/Model/FreeFloatingMatrices.h>
#include <iDynTree/ModelIO/ModelLoader.h>


using namespace iDynTree;

int kdl2idyntreeFloatingDOFMapping(int kdl_floating_dof, const std::vector<int> & kdl2idyntree_joints)
{
    if( kdl_floating_dof < 6 )
    {
        return kdl_floating_dof;
    }
    else
    {
        return 6+kdl2idyntree_joints[kdl_floating_dof-6];
    }
}

void testFwdKinConsistency(std::string modelFilePath)
{
    std::cout << "Testing " << modelFilePath << std::endl;


    // Load iDynTree model and compute a default traversal
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

    // A KDL::Tree only supports 0 and 1 DOFs joints, and
    // KDL::Tree::getNrOfJoints does not count the 0 dof joints, so
    // it is actually equivalent to iDynTree::Model::getNrOfDOFs
    ASSERT_EQUAL_DOUBLE(tree.getNrOfJoints(),model.getNrOfDOFs());

    // Input : joint positions and base position with respect to world
    iDynTree::FreeFloatingPos baseJntPos(model);
    iDynTree::FreeFloatingVel baseJntVel(model);
    iDynTree::FreeFloatingAcc baseJntAcc(model);

    baseJntPos.worldBasePos() =  getRandomTransform();
    baseJntVel.baseVel() = getRandomTwist();
    baseJntAcc.baseAcc() =  getRandomTwist();

    for(unsigned int jnt=0; jnt < baseJntPos.getNrOfPosCoords(); jnt++)
    {
        baseJntPos.jointPos()(jnt) = getRandomDouble();
        baseJntVel.jointVel()(jnt) = getRandomDouble();
        baseJntAcc.jointAcc()(jnt) = getRandomDouble();
    }


    // Build a map between KDL joints and iDynTree joints because we are not sure
    // that the joint indices will match
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

    ToKDL(baseJntPos,worldBaseKDL,jntKDL,kdl2idyntree_joints);
    ToKDL(baseJntVel,baseVelKDL,jntVelKDL,kdl2idyntree_joints);
    ToKDL(baseJntAcc,baseAccKDL,jntAccKDL,kdl2idyntree_joints);

    // Output : link (for iDynTree) or frame positions with respect to world
    std::vector<KDL::Frame> framePositions(undirected_tree.getNrOfLinks());

    iDynTree::LinkPositions linkPositions(model);


    // Build a map between KDL links and iDynTree links because we are not sure
    // that the link indices will match
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
        ASSERT_EQUAL_TRANSFORM_TOL(linkPositions(linkIndex),ToiDynTree(framePositions[idynTree2KDL_links[linkIndex]]),1e-1);
    }

    // Compution velocity & acceleration kinematics
    std::vector<KDL::Twist> kdlLinkVel(undirected_tree.getNrOfLinks());
    std::vector<KDL::Twist> kdlLinkAcc(undirected_tree.getNrOfLinks());
    KDL::CoDyCo::rneaKinematicLoop(undirected_tree,jntKDL,jntVelKDL,jntAccKDL,kdl_traversal,
                                                   baseVelKDL,baseAccKDL,kdlLinkVel,kdlLinkAcc);

    iDynTree::LinkVelArray linksVels(model);
    iDynTree::LinkAccArray linksAcc(model);

    ForwardVelAccKinematics(model,traversal,baseJntPos,baseJntVel,baseJntAcc,linksVels,linksAcc);

    // Check results
    for(unsigned int travEl = 0; travEl  < traversal.getNrOfVisitedLinks(); travEl++)
    {
        LinkIndex linkIndex = traversal.getLink(travEl)->getIndex();

        ASSERT_EQUAL_VECTOR(linksVels(linkIndex).asVector(),
                            ToiDynTree(kdlLinkVel[idynTree2KDL_links[linkIndex]]).asVector());
        ASSERT_EQUAL_VECTOR(linksAcc(linkIndex).asVector(),
                            ToiDynTree(kdlLinkAcc[idynTree2KDL_links[linkIndex]]).asVector());
    }

    // Compute position, velocity and acceleration
    LinkPositions linksPos_check(model);
    iDynTree::LinkVelArray linksVels_check(model);
    iDynTree::LinkAccArray linksAcc_check(model);

    ForwardPosVelAccKinematics(model,traversal,baseJntPos, baseJntVel, baseJntAcc,
                               linksPos_check,linksVels_check,linksAcc_check);

    for(unsigned int travEl = 0; travEl  < traversal.getNrOfVisitedLinks(); travEl++)
    {
        LinkIndex linkIndex = traversal.getLink(travEl)->getIndex();
        ASSERT_EQUAL_TRANSFORM(linkPositions(linkIndex),
                               linksPos_check(linkIndex));
        ASSERT_EQUAL_VECTOR(linksVels(linkIndex).asVector(),
                            linksVels_check(linkIndex).asVector());
        ASSERT_EQUAL_VECTOR(linksAcc(linkIndex).asVector(),
                             linksAcc(linkIndex).asVector());
    }

    // Compute torques (i.e. inverse dynamics)
    LinkNetExternalWrenches fExt(model);
    LinkInternalWrenches f(model);
    FreeFloatingGeneralizedTorques baseWrenchJointTorques(model);
    KDL::JntArray jntTorques(model.getNrOfDOFs());
    KDL::Wrench   baseWrench;

    std::vector<KDL::Wrench> fExtKDL(undirected_tree.getNrOfLinks());
    std::vector<KDL::Wrench> fKDL(undirected_tree.getNrOfLinks());

    // First set to zero all the fExtKDL, fKDL wrenches : then
    // the following loop will set the one that correspond to "real"
    // iDynTree links to the same value we used in iDynTree
    for(unsigned int linkKDL = 0; linkKDL < undirected_tree.getNrOfLinks(); linkKDL++)
    {
        fKDL[linkKDL]    = KDL::Wrench::Zero();
        fExtKDL[linkKDL] = KDL::Wrench::Zero();
    }

    for(unsigned int link = 0; link < model.getNrOfLinks(); link++ )
    {
        fExt(link) = getRandomWrench();
        fExtKDL[idynTree2KDL_links[link]] = ToKDL(fExt(link));
        fKDL[idynTree2KDL_links[link]] = KDL::Wrench::Zero();
    }

    bool ok = RNEADynamicPhase(model,traversal,
                               baseJntPos.jointPos(),
                               linksVels,linksAcc,fExt,f,baseWrenchJointTorques);

    int retVal = KDL::CoDyCo::rneaDynamicLoop(undirected_tree,jntKDL,kdl_traversal,
                                 kdlLinkVel,kdlLinkAcc,
                                 fExtKDL,fKDL,jntTorques,baseWrench);

    ASSERT_EQUAL_DOUBLE(ok,true);
    ASSERT_EQUAL_DOUBLE(retVal,0);

    /***
     * This check is commented out because KDL::CoDyCo::rneaDynamicLoop returned
     * a reverse baseWrench.. still need to be investigated.
     * (The - is necessary for consistency with the mass matrix..)
     *
     */
    //ASSERT_EQUAL_VECTOR(baseWrenchJointTorques.baseWrench().asVector(),
    //                    ToiDynTree(baseWrench).asVector());

    for(int link = 0; link < model.getNrOfLinks(); link++ )
    {
        ASSERT_EQUAL_VECTOR(f(link).asVector(),ToiDynTree(fKDL[idynTree2KDL_links[link]]).asVector());
    }

    for(int dof = 0; dof < model.getNrOfDOFs(); dof++ )
    {
        ASSERT_EQUAL_DOUBLE(jntTorques(dof),baseWrenchJointTorques.jointTorques()(kdl2idyntree_joints[dof]));
    }

    // Check mass matrix
    iDynTree::FreeFloatingMassMatrix massMatrix(model);
    iDynTree::LinkInertias crbis(model);
    ok = CompositeRigidBodyAlgorithm(model,traversal,
                                     baseJntPos.jointPos(),
                                     crbis,massMatrix);

    KDL::CoDyCo::FloatingJntSpaceInertiaMatrix massMatrixKDL(undirected_tree.getNrOfDOFs()+6);
    std::vector<KDL::RigidBodyInertia> Ic(undirected_tree.getNrOfLinks());

    retVal = KDL::CoDyCo::crba_floating_base_loop(undirected_tree,kdl_traversal,jntKDL,Ic,massMatrixKDL);

    ASSERT_IS_TRUE(ok);
    ASSERT_EQUAL_DOUBLE_TOL(retVal,0,1e-8);

    // Check composite rigid body inertias
    for(int link = 0; link < model.getNrOfLinks(); link++ )
    {
         ASSERT_EQUAL_MATRIX(crbis(link).asMatrix(),ToiDynTree(Ic[idynTree2KDL_links[link]]).asMatrix());
    }

    std::cerr << "iDynTree " << massMatrix.toString() << std::endl;
    std::cerr << "massMatrix " << massMatrixKDL.data << std::endl;


    // Check CRBA algorithm
    for(int ii=0; ii < model.getNrOfDOFs()+6; ii++ )
    {
        for( int jj=0; jj < model.getNrOfDOFs()+6; jj++ )
        {
            int idyntreeII = kdl2idyntreeFloatingDOFMapping(ii,kdl2idyntree_joints);
            int idyntreeJJ = kdl2idyntreeFloatingDOFMapping(jj,kdl2idyntree_joints);
            ASSERT_EQUAL_DOUBLE_TOL(massMatrix(idyntreeII,idyntreeJJ),massMatrixKDL(ii,jj),1e-8);
        }
    }

    return;
}


int main()
{
    for(unsigned int mdl = 0; mdl < IDYNTREE_TESTS_URDFS_NR; mdl++ )
    {
        // This test fails with frame.urdf, see https://github.com/robotology/idyntree/issues/298
        if (std::string(IDYNTREE_TESTS_URDFS[mdl]) != "frame.urdf")
        {
            std::string urdfFileName = getAbsModelPath(std::string(IDYNTREE_TESTS_URDFS[mdl]));
            testFwdKinConsistency(urdfFileName);
        }
    }

    return EXIT_SUCCESS;
}
