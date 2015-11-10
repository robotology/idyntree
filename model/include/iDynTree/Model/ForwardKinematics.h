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
    class FreeFloatingPosVelAcc;
    class LinkPositions;
    class LinkVelAccArray;
    class LinkPosVelAccArray;

    /**
     * Function that computes, given a traversal
     * the position forward kinematics of a FreeFloating robot.
     *
     * \ingroup iDynTreeModel
     */
    bool ForwardPositionKinematics(const iDynTree::Model & model,
                                   const iDynTree::Traversal & traversal,
                                   const iDynTree::FreeFloatingPos & jointPos,
                                   iDynTree::LinkPositions   & linkPos);


    /**
     * Function that compute the links velocities and accelerations
     * given the free floating robot velocities and accelerations.
     *
     * This function impelments what is usually known as the
     * "forward pass" of the Recursive Newton Euler algorithm. 
     */
    bool ForwardVelAccKinematics(const iDynTree::Model & model,
                                 const iDynTree::Traversal & traversal,
                                 const iDynTree::FreeFloatingPosVelAcc & jointPosVelAcc,
                                 iDynTree::LinkVelAccArray & linkPos);

    /**
     * Function that compute the links velocities and accelerations
     * given the free floating robot velocities and accelerations.
     *
     *
     */
    bool ForwardPosVelAccKinematics(const iDynTree::Model & model,
                                    const iDynTree::Traversal & traversal,
                                    const iDynTree::FreeFloatingPosVelAcc & jointPosVelAcc,
                                    iDynTree::LinkPosVelAccArray & linkPos);

}

#endif /* IDYNTREE_FORWARD_KINEMATICS_H */