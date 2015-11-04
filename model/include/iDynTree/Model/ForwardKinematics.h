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
    class LinkPositions;

    /**
     * Function that computes, given a traversal
     * the position forward kinematics of a FreeFloating robot.
     *
     * \ingroup iDynTreeModel
     */
    bool ForwardKinematics(const iDynTree::Model & model,
                      const iDynTree::Traversal & traversal,
                      const iDynTree::FreeFloatingPos & jointPos,
                       iDynTree::LinkPositions   & linkPos);

}

#endif /* IDYNTREE_FORWARD_KINEMATICS_H */