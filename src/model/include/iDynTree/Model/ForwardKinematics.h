/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_FORWARD_KINEMATICS_H
#define IDYNTREE_FORWARD_KINEMATICS_H

#include <iDynTree/Model/Indeces.h>

namespace iDynTree
{
    class Model;
    class Traversal;
    class Transform;
    class FreeFloatingPos;
    class FreeFloatingVel;
    class FreeFloatingAcc;
    class LinkPositions;
    class LinkVelArray;
    class LinkAccArray;
    class JointPosDoubleArray;

    /**
     * \ingroup iDynTreeModel
     *
     * Given a robot floating base configuration (i.e. world_H_base and the joint positions)
     * compute for each link the world_H_link transform.
     *
     * @param[in]  model the used model,
     * @param[in]  traversal the used traversal,
     * @param[in]  worldHbase the world_H_base transform,
     * @param[in]  jointPositions the vector of (internal) joint positions,
     * @param[out] linkPositions linkPositions(l) contains the world_H_link transform.
     * @return true if all went well, false otherwise.
     */
    bool ForwardPositionKinematics(const Model& model,
                                   const Traversal& traversal,
                                   const Transform& worldHbase,
                                   const JointPosDoubleArray& jointPositions,
                                         LinkPositions& linkPositions);

    /**
     * Variant of ForwardPositionKinematics that takes in input a FreeFloatingPos
     * object instead of a separate couple of (worldHbase,jointPositions)
     *
     */
    bool ForwardPositionKinematics(const Model & model,
                                   const Traversal & traversal,
                                   const FreeFloatingPos & jointPos,
                                         LinkPositions   & linkPos);



    /**
     * Function that compute the links velocities and accelerations
     * given the free floating robot velocities and accelerations.
     *
     * This function impelments what is usually known as the
     * "forward pass" of the Recursive Newton Euler algorithm.
     */
    bool ForwardVelAccKinematics(const iDynTree::Model & model,
                                 const iDynTree::Traversal & traversal,
                                 const iDynTree::FreeFloatingPos & robotPos,
                                 const iDynTree::FreeFloatingVel & robotVel,
                                 const iDynTree::FreeFloatingAcc & robotAcc,
                                       iDynTree::LinkVelArray & linkVel,
                                       iDynTree::LinkAccArray  & linkAcc);

    /**
     * Function that compute the links position, velocities and accelerations
     * given the free floating robot position, velocities and accelerations.
     *
     *
     */
    bool ForwardPosVelAccKinematics(const iDynTree::Model & model,
                                    const iDynTree::Traversal & traversal,
                                    const iDynTree::FreeFloatingPos & robotPos,
                                    const iDynTree::FreeFloatingVel & robotVel,
                                    const iDynTree::FreeFloatingAcc & robotAcc,
                                          iDynTree::LinkPositions & linkPos,
                                          iDynTree::LinkVelArray & linkVel,
                                          iDynTree::LinkAccArray  & linkAcc);

    /**
     * Function that compute the links position and velocities and accelerations
     * given the free floating robot position and velocities.
     *
     *
     */
    bool ForwardPosVelKinematics(const iDynTree::Model & model,
                                 const iDynTree::Traversal & traversal,
                                 const iDynTree::FreeFloatingPos & robotPos,
                                 const iDynTree::FreeFloatingVel & robotVel,
                                       iDynTree::LinkPositions & linkPos,
                                       iDynTree::LinkVelArray & linkVel);

}

#endif
