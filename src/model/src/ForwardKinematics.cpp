// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause


#include <iDynTree/ForwardKinematics.h>

#include <iDynTree/Model.h>
#include <iDynTree/Traversal.h>

#include <iDynTree/FreeFloatingState.h>
#include <iDynTree/LinkState.h>

namespace iDynTree
{

bool ForwardPositionKinematics(const Model& model,
                               const Traversal& traversal,
                               const FreeFloatingPos& robotPosition,
                                     LinkPositions& linkPositions)
{
    return ForwardPositionKinematics(model,traversal,
                                     robotPosition.worldBasePos(),robotPosition.jointPos(),
                                     linkPositions);
}

bool ForwardPositionKinematics(const Model& model,
                               const Traversal& traversal,
                               const Transform& worldHbase,
                               const VectorDynSize& jointPositions,
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
            linkPositions(visitedLink->getIndex()) =worldHbase;
        }
        else
        {
            // Otherwise we compute the world_H_link transform as:
            // world_H_link = world_H_parentLink * parentLink_H_link
            linkPositions(visitedLink->getIndex()) =
                linkPositions(parentLink->getIndex())*
                    toParentJoint->getTransform(jointPositions,parentLink->getIndex(),visitedLink->getIndex());
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
                                              visitedLink->getIndex(),
                                              parentLink->getIndex());
        }

    }

    return retValue;

}

bool ForwardPosVelKinematics(const Model& model,
                             const Traversal& traversal,
                             const FreeFloatingPos& robotPos,
                             const FreeFloatingVel& robotVel,
                                   LinkPositions& linkPos,
                                   LinkVelArray& linkVel)
{
    bool retValue = true;

    for (TraversalIndex traversalEl=0; traversalEl < traversal.getNrOfVisitedLinks(); traversalEl++)
    {
        LinkConstPtr visitedLink = traversal.getLink(traversalEl);
        LinkConstPtr parentLink  = traversal.getParentLink(traversalEl);
        IJointConstPtr toParentJoint = traversal.getParentJoint(traversalEl);

        if (parentLink == 0)
        {
            // If the visited link is the base, the base has no parent.
            // In this case the position of the base with respect to the world is simply
            // the worldBase transform contained in the jointPos input object and the velocity
            // of the base is the base velocity
            linkPos(visitedLink->getIndex()) = robotPos.worldBasePos();
            linkVel(visitedLink->getIndex()) = robotVel.baseVel();
        }
        else
        {
            // Otherwise we compute the world_H_link transform as:
            // world_H_link = world_H_parentLink * parentLink_H_link
            linkPos(visitedLink->getIndex()) =
                linkPos(parentLink->getIndex()) *
                    toParentJoint->getTransform(robotPos.jointPos(),parentLink->getIndex(),visitedLink->getIndex());

            // The link velocity are recursivly
            // compute from the joint position, velocities
            // and from the velocity of the parent link
            toParentJoint->computeChildVel(robotPos.jointPos(),
                                           robotVel.jointVel(),
                                           linkVel,
                                           visitedLink->getIndex(),parentLink->getIndex());
        }
    }

    return retValue;
}

bool ForwardAccKinematics(const Model& model,
                          const Traversal& traversal,
                          const FreeFloatingPos & robotPos,
                          const FreeFloatingVel & robotVel,
                          const iDynTree::FreeFloatingAcc & robotAcc,
                          const LinkVelArray & linkVel,
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
            // In this case the acceleration is (by definition) part of robotAcc
            linkAcc(visitedLink->getIndex()) = robotAcc.baseAcc();
        }
        else
        {
            // Otherwise we compute the child velocity and acceleration from parent
            toParentJoint->computeChildAcc(robotPos.jointPos(),
                                           robotVel.jointVel(),
                                           linkVel,
                                           robotAcc.jointAcc(),
                                           linkAcc,
                                           visitedLink->getIndex(),
                                           parentLink->getIndex());
        }

    }

    return retValue;
}

bool ForwardBiasAccKinematics(const Model& model,
                              const Traversal& traversal,
                              const FreeFloatingPos & robotPos,
                              const FreeFloatingVel & robotVel,
                              const SpatialAcc& baseBiasAcc,
                              const LinkVelArray & linkVel,
                                    LinkAccArray & linkBiasAcc)
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
            // In this case the bias acceleration is tipically zero,
            // or in strange cases the one specified by the baseBiasAcc argument
            linkBiasAcc(visitedLink->getIndex()) = baseBiasAcc;
        }
        else
        {
            // Otherwise we compute the child velocity and acceleration from parent
            toParentJoint->computeChildBiasAcc(robotPos.jointPos(),
                                               robotVel.jointVel(),
                                               linkVel, linkBiasAcc,
                                               visitedLink->getIndex(),
                                               parentLink->getIndex());
        }

    }

    return retValue;
}

bool ForwardBiasAccKinematics(const Model& model,
                              const Traversal& traversal,
                              const FreeFloatingPos & robotPos,
                              const FreeFloatingVel & robotVel,
                              const LinkVelArray & linkVel,
                                    LinkAccArray & linkBiasAcc)
{
    return ForwardBiasAccKinematics(model, traversal, robotPos, robotVel,
                                    SpatialAcc::Zero(), linkVel, linkBiasAcc);
}

}
