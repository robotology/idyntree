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

bool ForwardKinematics(const Model& model,
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
            linkPositions.linkPos(visitedLink->getIndex()).pos() = jointPositions.worldBasePos();
        }
        else
        {
            // Otherwise we compute the world_H_link transform as:
            // world_H_link = world_H_parentLink * parentLink_H_link
            linkPositions.linkPos(visitedLink->getIndex()).pos() =
                linkPositions.linkPos(parentLink->getIndex()).pos()*
                    toParentJoint->getTransform(jointPositions.jointPos(toParentJoint->getIndex()),parentLink->getIndex(),visitedLink->getIndex());
        }
    }

    return retValue;
}

}
