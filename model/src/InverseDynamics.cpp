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
#include <iDynTree/Model/LinkState.h>

#include <iDynTree/Core/SpatialMomentum.h>

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


}
