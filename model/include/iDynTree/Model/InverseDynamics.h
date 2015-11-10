/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_INVERSE_DYNAMICS_H
#define IDYNTREE_INVERSE_DYNAMICS_H

#include <iDynTree/Model/Indeces.h>

#include <iDynTree/Model/LinkState.h>

namespace iDynTree
{
    class Model;
    class Traversal;
    class FreeFloatingPos;
    class FreeFloatingPosVel;
    class FreeFloatingPosVelAcc;
    class FreeFloatingAcc;
    class FreeFloatingGeneralizedTorques;
    class FreeFloatingMassMatrix;
    class JointDoubleArray;
    class DOFSpatialForceArray;
    class DOFSpatialMotionArray;

    bool RNEADynamicPhase(const iDynTree::Model & model,
                          const iDynTree::Traversal & traversal,
                          const iDynTree::FreeFloatingPosVelAcc & jointPosVelAcc,
                          const iDynTree::LinkVelAccArray & linksVelAccs,
                          const iDynTree::LinkExternalWrenches & linkExtForces,
                          iDynTree::LinkInternalWrenches       & linkIntWrenches,
                          iDynTree::FreeFloatingGeneralizedTorques & baseForceAndJointTorques);

    /**
     * Compute the floating base mass matrix, using the
     * composite rigid body algorithm.
     *
     */
    bool CompositeRigidBodyAlgorithm(const Model& model,
                                     const Traversal& traversal,
                                     const FreeFloatingPos& jointPos,
                                     LinkCompositeRigidBodyInertias& linkCRBs,
                                     FreeFloatingMassMatrix& massMatrix);

    /**
     * Compute the floating base acceleration of an unconstrianed
     * robot, using as input the external forces and the joint torques.
     * We follow the algorithm described in Featherstone 2008, modified
     * for the floating base case and for handling fixed joints.
     *
     */
    bool ArticulatedBodyAlgorithm(const Model& model,
                                  const Traversal& traversal,
                                  const FreeFloatingPosVel& robotPosVel,
                                  const LinkExternalWrenches & linkExtWrenches,
                                  const JointDoubleArray & jointTorques,
                                        DOFSpatialMotionArray & S,
                                        DOFSpatialForceArray & U,
                                        JointDoubleArray & D,
                                        JointDoubleArray & u,
                                        LinkVelArray & linksVel,
                                        LinkAccArray & linksBiasAcceleration,
                                        LinkAccArray & linksAccelerations,
                                        LinkArticulatedBodyInertias & linkABIs,
                                        LinkWrenches & linksBiasWrench,
                                        FreeFloatingAcc & robotAcc);

}


#endif /* IDYNTREE_DYNAMICS_H */