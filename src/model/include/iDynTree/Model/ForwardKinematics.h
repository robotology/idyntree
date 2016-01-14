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
    class FreeFloatingPos;
    class FreeFloatingVel;
    class FreeFloatingAcc;
    class LinkPositions;
    class LinkVelArray;
    class LinkAccArray;

    /**
     * Function that computes, given a traversal
     * the position forward kinematics of a FreeFloating robot.
     *
     * \ingroup iDynTreeModel
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
     * Function that compute the links velocities and accelerations
     * given the free floating robot velocities and accelerations.
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

}

#endif /* IDYNTREE_FORWARD_KINEMATICS_H */