/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


#include <iDynTree/Model/ForwardKinematics.h>

#include <iDynTree/Model/Model.h>
#include <iDynTree/Model/Traversal.h>

#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/Model/LinkState.h>

namespace iDynTree
{

bool ForwardPositionKinematics(const Model& model,
                               const Traversal& traversal,
                               const FreeFloatingPos& jointPositions,
                                     LinkPositions& linkPositions)
{
    bool retValue = true;

    for(unsigned int traversalEl=0; traversalEl < traversal.getNrOfVisitedLinks(); traversalEl++)
    {
        LinkConstPtr visitedLink = traversal.getLink(traversalEl);
        LinkConstPtr parentLink  = traversal.getParentLink(traversalEl);
        IJointConstPtr toParentJoint = traversal.getParentJoint(traversalEl);

        if( parentLink == 0 )
        {
            // If the visited link is the base, the base has no parent.
            // In this case the position of the base with respect to the world is simply
            // the worldBase transform contained in the jointPos input object
            linkPositions(visitedLink->getIndex()) = jointPositions.worldBasePos();
        }
        else
        {
            // Otherwise we compute the world_H_link transform as:
            // world_H_link = world_H_parentLink * parentLink_H_link
            linkPositions(visitedLink->getIndex()) =
                linkPositions(parentLink->getIndex())*
                    toParentJoint->getTransform(jointPositions.jointPos(),parentLink->getIndex(),visitedLink->getIndex());
        }
    }

    return retValue;
}

bool ForwardPosVelAccKinematics(const Model& model, const Traversal& traversal,
                                const FreeFloatingPos& robotPos,
                                const FreeFloatingVel& robotVel,
                                const FreeFloatingAcc& robotAcc,
                                LinkPositions& linkPos,
                                LinkVelArray & linkVel,
                                LinkAccArray & linkAcc)
{
    bool retValue = true;

    for(unsigned int traversalEl=0; traversalEl < traversal.getNrOfVisitedLinks(); traversalEl++)
    {
        LinkConstPtr visitedLink = traversal.getLink(traversalEl);
        LinkConstPtr parentLink  = traversal.getParentLink(traversalEl);
        IJointConstPtr toParentJoint = traversal.getParentJoint(traversalEl);

        if( parentLink == 0 )
        {
            // If the visited link is the base, the base has no parent.
            // In this case the position of the base with respect to the world is simply
            // the worldBase transform contained in the jointPos input object
            linkPos(visitedLink->getIndex()) = robotPos.worldBasePos();
            linkVel(visitedLink->getIndex()) = robotVel.baseVel();
            linkAcc(visitedLink->getIndex()) = robotAcc.baseAcc();
        }
        else
        {
            // Otherwise we compute the world_H_link transform as:
            // world_H_link = world_H_parentLink * parentLink_H_link
            // The link velocity and acceleration are recursivly
            // compute from the joint position, velocity and acceleration
            // and from the velocity and acceleration of the parent link
            toParentJoint->computeChildPosVelAcc(robotPos.jointPos(),
                                                 robotVel.jointVel(),
                                                 robotAcc.jointAcc(),
                                                 linkPos, linkVel, linkAcc,
                                                 visitedLink->getIndex(),parentLink->getIndex());
        }
    }

    return retValue;

}

bool ForwardVelAccKinematics(const Model& model, const Traversal& traversal,
                                 const iDynTree::FreeFloatingPos & robotPos,
                                 const iDynTree::FreeFloatingVel & robotVel,
                                 const iDynTree::FreeFloatingAcc & robotAcc,
                                       iDynTree::LinkVelArray & linkVel,
                                       iDynTree::LinkAccArray & linkAcc)
{
    bool retValue = true;

    for(unsigned int traversalEl=0; traversalEl < traversal.getNrOfVisitedLinks(); traversalEl++)
    {
        LinkConstPtr visitedLink = traversal.getLink(traversalEl);
        LinkConstPtr parentLink  = traversal.getParentLink(traversalEl);
        IJointConstPtr toParentJoint = traversal.getParentJoint(traversalEl);

        if( parentLink == 0 )
        {
            // If the visited link is the base, the base has no parent.
            // In this case the position of the base with respect to the world is simply
            // the worldBase transform contained in the jointPos input object
            linkVel(visitedLink->getIndex()) = robotVel.baseVel();
            linkAcc(visitedLink->getIndex()) = robotAcc.baseAcc();
        }
        else
        {
            // Otherwise we compute the child velocity and acceleration from parent
            toParentJoint->computeChildVelAcc(robotPos.jointPos(),
                                              robotVel.jointVel(),
                                              robotAcc.jointAcc(),
                                              linkVel, linkAcc,
                                              visitedLink->getIndex(),parentLink->getIndex());
        }

    }

    return retValue;

}



}
