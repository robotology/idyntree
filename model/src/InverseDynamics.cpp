/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


#include <iDynTree/Model/InverseDynamics.h>

#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/Traversal.h>

#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/Model/FreeFloatingMassMatrix.h>
#include <iDynTree/Model/LinkState.h>

#include <iDynTree/Core/SpatialMomentum.h>
#include <iDynTree/Core/EigenHelpers.h>

#include <Eigen/Core>

namespace iDynTree
{

bool RNEADynamicPhase(const Model& model, const Traversal& traversal,
                      const FreeFloatingPosVelAcc& jointPosVelAcc,
                      const LinkVelAccArray& linksVelAccs,
                      const LinkExternalWrenches& fext,
                      LinkInternalWrenches& f,
                      FreeFloatingGeneralizedTorques& baseWrenchJntTorques)
{
    bool retValue = true;

    for(int traversalEl = traversal.getNrOfVisitedLinks()-1; traversalEl >= 0; traversalEl--)
    {
        LinkConstPtr visitedLink = traversal.getLink(traversalEl);
        LinkIndex    visitedLinkIndex = visitedLink->getIndex();
        LinkConstPtr parentLink  = traversal.getParentLink(traversalEl);
        IJointConstPtr toParentJoint = traversal.getParentJoint(traversalEl);

        // We sum in the link internal wrench the inertial wrenches,
        // the external wrenches and the child link wrenches .
        // It is Equation 5.20 in Featherstone 2008 , with the
        // only difference that we assume the external forces
        // are expressed in the link reference frame (both orientation
        // and point).
        const iDynTree::SpatialInertia & I = visitedLink->getInertia();
        const iDynTree::SpatialAcc     & a = linksVelAccs.linkVelAcc(visitedLinkIndex).acc();
        const iDynTree::Twist          & v = linksVelAccs.linkVelAcc(visitedLinkIndex).vel();
        f(visitedLinkIndex) = I*a + v*(I*v) - fext(visitedLinkIndex);

        // Iterate on childs of visitedLink
        // We obtain all the children as all the neighbors of the link, except
        // for its parent
        // \todo TODO this point is definitly Tree-specific
        // \todo TODO this "get child" for is duplicated in the code, we
        //            should try to consolidate it
        for(int neigh_i=0; neigh_i < model.getNrOfNeighbors(visitedLinkIndex); neigh_i++)
        {
             LinkIndex neighborIndex = model.getNeighbor(visitedLinkIndex,neigh_i).neighborLink;
             if( !parentLink || neighborIndex != parentLink->getIndex() )
             {
                 LinkIndex childIndex = neighborIndex;
                 IJointConstPtr neighborJoint = model.getJoint(model.getNeighbor(visitedLinkIndex,neigh_i).neighborJoint);
                 Transform visitedLink_X_child = neighborJoint->getTransform(jointPosVelAcc.jointPos(),visitedLinkIndex,childIndex);

                 // One term of the sum in Equation 5.20 in Featherstone 2008
                 f(visitedLinkIndex) = f(visitedLinkIndex) + visitedLink_X_child*f(childIndex);
             }
        }

        if( parentLink == 0 )
        {
            // If the visited link is the base, the base has no parent, and hence no
            // joint torque to compute.
            // In this case the base wrench is simply saved in the output generalized
            // torques vector (with a minus because the base force is the one applied
            // on the base, while f[visitedLinkIndex] stores the one applied by the base.
            // Notice that this force, if the model, the accelerations and the external wrenches
            // are coherent, should be zero. This because any external wrench on the base should
            // be present also in the fExt vector .
            baseWrenchJntTorques.baseWrench() = -f(visitedLinkIndex);
        }
        else
        {
            // If the visited link is not the base and  connected to a parent link
            // at this point we can compute the torque of the joint connecting this link and its parent
            // This is Equation 5.13 in Featherstone 2008. It is offloaded to the joint to be
            // able to deal with different kind of joints.
            toParentJoint->computeJointTorque(jointPosVelAcc.jointPos(),
                                              f(visitedLinkIndex),
                                              parentLink->getIndex(),
                                              visitedLinkIndex,
                                              baseWrenchJntTorques.jointTorques());
        }
    }

    return retValue;
}

bool CompositeRigidBodyAlgorithm(const Model& model,
                                 const Traversal& traversal,
                                 const FreeFloatingPos& jointPos,
                                 LinkCompositeRigidBodyInertias& linkCRBs,
                                 FreeFloatingMassMatrix& massMatrix)
{
    // Map the massMatrix to an Eigen matrix
    Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> >
        massMatrixEigen(massMatrix.data(),massMatrix.rows(),massMatrix.cols());

    /**
     * Forward pass: initialize the CRBI
     * of each link to its own inertia.
     */
    for(unsigned int traversalEl=0; traversalEl < traversal.getNrOfVisitedLinks(); traversalEl++)
    {
        LinkConstPtr visitedLink = traversal.getLink(traversalEl);
        LinkIndex visitedLinkIndex = visitedLink->getIndex();
        linkCRBs(visitedLinkIndex) = visitedLink->getInertia();
    }

    /**
     * Backward pass: for each link compute the
     * CRB of the link (given the traversal).
     */
    for(int traversalEl = traversal.getNrOfVisitedLinks()-1; traversalEl >= 0; traversalEl--)
    {
        LinkConstPtr visitedLink = traversal.getLink(traversalEl);
        LinkIndex    visitedLinkIndex = visitedLink->getIndex();
        LinkConstPtr parentLink  = traversal.getParentLink(traversalEl);
        IJointConstPtr toParentJoint = traversal.getParentJoint(traversalEl);

        // If the visited link is not the base one, add its
        // CRBI to the CRBI of the parent
        // \todo TODO streamline the check "is Link the floating base"
        // given a traversal
        if( parentLink )
        {
            LinkIndex parentLinkIndex = parentLink->getIndex();

            linkCRBs(parentLinkIndex) = linkCRBs(parentLinkIndex) +
                (toParentJoint->getTransform(jointPos.jointPos(),parentLinkIndex,visitedLinkIndex))*linkCRBs(visitedLinkIndex);

            // For now we just implement the CRBA for 0 or 1 dofs joints.
            assert( toParentJoint->getNrOfDOFs() <= 1 );

            // If the visited link is attached to its parent with a fixed joint,
            // we don't need to do anything else for this link.
            // Otherwise we need to compute the rows and columns of the mass matrix
            // related to the dof connecting the visited link to its parent
            if( toParentJoint->getNrOfDOFs() == 1 )
            {
                // In this loop we follow the algorith as described in
                // in Featherstone 2008 , Table 6.2 . In particular S_visitedDof (S_i in the book)
                // is the motion subspace vector connected to the degree of freedom connecting the link to its parent,
                // while S_ancestorDof (S_j in the book) is the motion subspace vector of its ancestor considered in the
                // inner loop
                SpatialMotionVector S_visitedDof = toParentJoint->getMotionSubspaceVector(0,visitedLink->getIndex(),parentLinkIndex);
                SpatialForceVector  F = linkCRBs(visitedLinkIndex)*S_visitedDof;

                // We compute the term of the mass matrix on the diagonal
                // (in the book: H_ii = S_i^\top F
                size_t dofIndex = toParentJoint->getDOFsOffset();
                massMatrix(6+dofIndex,6+dofIndex) = S_visitedDof.dot(F);

                // Then we compute all the off-diagonal terms relative to
                // the ancestors of the currently visited link

                // j = i
                LinkConstPtr ancestor = visitedLink;

                // while lambda(j) != 0
                while( traversal.getParentLinkFromLinkIndex(traversal.getParentLinkFromLinkIndex(ancestor->getIndex())->getIndex()) )
                {
                    {
                        IJointConstPtr ancestorToParentJoint = traversal.getParentJointFromLinkIndex(ancestor->getIndex());
                        LinkIndex      ancestorParent =        traversal.getParentLinkFromLinkIndex(ancestor->getIndex())->getIndex();
                        Transform ancestorParent_X_ancestor = ancestorToParentJoint->getTransform(jointPos.jointPos(),ancestorParent,ancestor->getIndex());
                        F = ancestorParent_X_ancestor*F;
                    }

                    // j = \lambda(j)

                    ancestor = traversal.getParentLinkFromLinkIndex(ancestor->getIndex());

                    IJointConstPtr ancestorToParentJoint = traversal.getParentJointFromLinkIndex(ancestor->getIndex());
                    LinkIndex      ancestorParentIndex   = traversal.getParentLinkFromLinkIndex(ancestor->getIndex())->getIndex();

                    // For now we just implement the CRBA for 0 or 1 dofs joints.
                    assert( ancestorToParentJoint->getNrOfDOFs() <= 1 );

                    if( ancestorToParentJoint->getNrOfDOFs() == 1 )
                    {
                        SpatialMotionVector S_ancestorDof =
                            ancestorToParentJoint->getMotionSubspaceVector(0,ancestor->getIndex(),ancestorParentIndex);
                        size_t ancestorDofIndex = ancestorToParentJoint->getDOFsOffset();

                        // H_ij = F^\top S_j
                        // H_ji = H_ij^\top
                        massMatrix(6+dofIndex,6+ancestorDofIndex) = S_ancestorDof.dot(F);
                        massMatrix(6+ancestorDofIndex,6+dofIndex) = massMatrix(6+dofIndex,6+ancestorDofIndex);
                    }
                }

                // We need to write F in the base link for the off-diagonal term of the mass matrix
                {
                    IJointConstPtr ancestorToParentJoint = traversal.getParentJointFromLinkIndex(ancestor->getIndex());
                    LinkIndex      ancestorParent =        traversal.getParentLinkFromLinkIndex(ancestor->getIndex())->getIndex();
                    Transform ancestorParent_X_ancestor = ancestorToParentJoint->getTransform(jointPos.jointPos(),ancestorParent,ancestor->getIndex());
                    F = ancestorParent_X_ancestor*F;
                }


                // Fill the 6 \times nDof right top submatrix of the mass matrix
                // (i.e. the jacobian of the momentum)
                Eigen::Matrix<double,6,1> FEigen = toEigen(F);
                massMatrixEigen.block<6,1>(0,6+dofIndex) = FEigen;
                massMatrixEigen.block<1,6>(6+dofIndex,0) = FEigen;
            }

        }
    }

    // Fill the top left 6x6 matrix: it is just the composite rigid body inertia of all the body
    Matrix6x6 lockedInertia = linkCRBs(traversal.getLink(0)->getIndex()).asMatrix();
    massMatrixEigen.block<6,6>(0,0) = toEigen(lockedInertia);

    return true;
}


}
