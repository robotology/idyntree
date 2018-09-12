/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_FORWARD_KINEMATICS_H
#define IDYNTREE_FORWARD_KINEMATICS_H

#include <iDynTree/Model/Indices.h>

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
    class SpatialAcc;
    class VectorDynSize;

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
                                   const VectorDynSize& jointPositions,
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
     * This function implements what is usually known as the
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

    /**
     * Function that computes the links accelerations
     * given the free floating robot velocities and accelerations.
     *
     * This function implements what is usually known as the acceleration part
     * "forward pass" of the Recursive Newton Euler algorithm.
     */
    bool ForwardAccKinematics(const Model & model,
                              const Traversal & traversal,
                              const FreeFloatingPos & robotPos,
                              const FreeFloatingVel & robotVel,
                              const FreeFloatingAcc & robotAcc,
                              const LinkVelArray & linkVel,
                                    LinkAccArray  & linkAcc);

    /**
     * Function that computes the links bias accelerations given the free floating robot velocities.
     *
     * @note This function can also consider a non-zero base acceleration, because when computing
     *        the bias acc for the MIXED representation, a zero mixed base acceleration is a equivalent
     *        to a non-zero BODY_FIXED base acceleration.
     *
     * @param[in]  model the used model,
     * @param[in]  traversal the used traversal,
     * @param[in]  worldHbase the world_H_base transform,
     * @param[in]  robotPos the position of the robot, i.e. \f$ ({}^A H_B, s)\f$,
     * @param[in]  robotVel the velocity of the robot, with the base velocity expressed in BODY_FIXED
     *                      representation i.e. \f$ \nu = \begin{bmatrix} {}^B \mathrm{v}_{A,B} \newline \dot{s} \end{bmatrix} \f$,
     * @param[in]  baseBiasAcc base bias acceleration with BODY_FIXED rapresentation, useful when the bias acceleration
     *                         is considering the MIXED base acceleration to be zero,
     * @param[in]  linkVel the velocity of each link of the robot, with velocity expressed with BODY_FIXED
     *                      representation i.e. for each link \f$ \f$  we have \f$ {}^L \mathrm{v}_{A,L} \f$,
     * @param[out]  linkBiasAcc the bias acceleration of each link of the robot, with the acceleration expressed with BODY_FIXED
     *                      representation.
     *
     */
    bool ForwardBiasAccKinematics(const Model & model,
                                  const Traversal & traversal,
                                  const FreeFloatingPos & robotPos,
                                  const FreeFloatingVel & robotVel,
                                  const SpatialAcc & baseBiasAcc,
                                  const LinkVelArray & linkVel,
                                        LinkAccArray  & linkBiasAcc);

    /**
     * Legacy function, will be  deprecated, use the variant with an explicit baseBiasAcc value.
     */
    bool ForwardBiasAccKinematics(const Model & model,
                                  const Traversal & traversal,
                                  const FreeFloatingPos & robotPos,
                                  const FreeFloatingVel & robotVel,
                                  const LinkVelArray & linkVel,
                                        LinkAccArray  & linkBiasAcc);

}

#endif
