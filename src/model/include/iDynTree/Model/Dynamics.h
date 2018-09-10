/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_INVERSE_DYNAMICS_H
#define IDYNTREE_INVERSE_DYNAMICS_H

#include <iDynTree/Core/MatrixDynSize.h>

#include <iDynTree/Model/Indices.h>
#include <iDynTree/Model/LinkState.h>
#include <iDynTree/Model/JointState.h>

namespace iDynTree
{
    class Model;
    class Traversal;
    class FreeFloatingPos;
    class FreeFloatingVel;
    class FreeFloatingAcc;
    class FreeFloatingGeneralizedTorques;
    class FreeFloatingMassMatrix;
    class JointDOFsDoubleArray;
    class DOFSpatialForceArray;
    class DOFSpatialMotionArray;
    class SpatialMomentum;

    /**
     * \ingroup iDynTreeModel
     *
     * Compute the total linear and angular momentum of a robot, expressed in the world frame.
     *
     * @param[in]  model the used model,
     * @param[in]  linkPositions linkPositions(l) contains the world_H_link transform.
     * @param[in]  linkVels linkVels(l) contains the link l velocity expressed in l frame.
     * @param[out] totalMomentum total momentum, expressed in world frame.
     * @return true if all went well, false otherwise.
     */
    bool ComputeLinearAndAngularMomentum(const Model& model,
                                         const LinkPositions& linkPositions,
                                         const LinkVelArray&  linkVels,
                                               SpatialMomentum& totalMomentum);

    /**
     * Compute the total momentum derivatitive bias, i.e. the part of the total momentum derivative that does not depend on robot acceleration.
     *
     * The linear and angular momentum derivative depends on the robot position, velocity and acceleration.
     * This function computes the part that do not depend on the robot accelearation.
     *
     * This function returns the bias of the derivative of the ComputeLinearAndAngularMomentum function.
     */
    bool ComputeLinearAndAngularMomentumDerivativeBias(const Model & model,
                                                       const LinkPositions& linkPositions,
                                                       const LinkVelArray & linkVel,
                                                       const LinkAccArray & linkBiasAcc,
                                                             Wrench& totalMomentumBias);

    /**
     * \ingroup iDynTreeModel
     *
     * @brief Compute the inverse dynamics, i.e. the generalized torques corresponding to a given set of robot accelerations and external force/torques.
     *
     * @param[in] model The model used for the computation.
     * @param[in] traversal The traversal used for the computation, it defines the used base link.
     * @param[in] jointPos The (internal) joint position of the model.
     * @param[in] linksVel Vector of left-trivialized velocities for each link of the model (for each link \f$L\f$, the corresponding velocity is \f${}^L \mathrm{v}_{A, L}\f$).
     * @param[in] linksProperAcc Vector of left-trivialized proper acceleration for each link of the model
     *                           (for each link \f$L\f$, the corresponding proper acceleration is \f${}^L \dot{\mathrm{v}}_{A, L} - \begin{bmatrix} {}^L R_A {}^A g \\ 0_{3\times1} \end{bmatrix} \f$), where \f$ {}^A g \in \mathbb{R}^3 \f$ is the gravity acceleration expressed in an inertial frame \f$A\f$ . See iDynTree::LinkNetExternalWrenches .
     * @param[in] linkExtForces Vector of external 6D force/torques applied to the links. For each link \f$L\f$, the corresponding external force is \f${}_L \mathrm{f}^x_L\f$, i.e. the force that the enviroment applies on the on the link \f$L\f$, expressed in the link frame \f$L\f$.
     * @param[out] linkIntWrenches Vector of internal joint force/torques. See iDynTree::LinkInternalWrenches .
     * @param[out] baseForceAndJointTorques Generalized torques output. The base element is the residual force on the base (that is equal to zero if the robot acceleration and the external forces provided in LinkNetExternalWrenches were consistent), while the joint part is composed by the joint torques.
     */
    bool RNEADynamicPhase(const iDynTree::Model & model,
                          const iDynTree::Traversal & traversal,
                          const iDynTree::JointPosDoubleArray & jointPos,
                          const iDynTree::LinkVelArray & linksVel,
                          const iDynTree::LinkAccArray & linksProperAcc,
                          const iDynTree::LinkNetExternalWrenches & linkExtForces,
                                iDynTree::LinkInternalWrenches       & linkIntWrenches,
                                iDynTree::FreeFloatingGeneralizedTorques & baseForceAndJointTorques);

    /**
     * Compute the floating base mass matrix, using the
     * composite rigid body algorithm.
     *
     */
    bool CompositeRigidBodyAlgorithm(const Model& model,
                                     const Traversal& traversal,
                                     const JointPosDoubleArray& jointPos,
                                     LinkCompositeRigidBodyInertias& linkCRBs,
                                     FreeFloatingMassMatrix& massMatrix);


    /**
     * Structure of buffers required by ArticulatedBodyAlgorithm.
     *
     * As the ArticulatedBodyAlgorithm function needs some internal buffers
     * to run, but we don't want to put memory allocation inside the ArticulatedBodyAlgorithm
     * function, we put all the internal buffers in this structure.
     *
     * A convenient resize(Model) function is provided to automatically resize
     * the buffers given a Model.
     */
    struct ArticulatedBodyAlgorithmInternalBuffers
    {
        ArticulatedBodyAlgorithmInternalBuffers() {};

        /**
         * Call resize(model);
         */
        ArticulatedBodyAlgorithmInternalBuffers(const Model & model);

        /**
         * Resize all the buffers to the right size given the model,
         * and reset all the buffers to 0.
         */
        void resize(const Model& model);

        /**
         * Check if the dimension of the buffer is consistent
         * with a model (it should be after a call to resize(model) ).
         */
        bool isConsistent(const Model& model);

        DOFSpatialMotionArray S;
        DOFSpatialForceArray U;
        JointDOFsDoubleArray D;
        JointDOFsDoubleArray u;
        LinkVelArray linksVel;
        LinkAccArray linksBiasAcceleration;
        LinkAccArray linksAccelerations;
        LinkArticulatedBodyInertias linkABIs;
        LinkWrenches linksBiasWrench;

        // Debug quantity
        //LinkWrenches pa;
    };

    /**
     * \ingroup iDynTreeModel
     *
     * Compute the floating base acceleration of an unconstrianed
     * robot, using as input the external forces and the joint torques.
     * We follow the algorithm described in Featherstone 2008, modified
     * for the floating base case and for handling fixed joints.
     *
     */
    bool ArticulatedBodyAlgorithm(const Model& model,
                                  const Traversal& traversal,
                                  const FreeFloatingPos& robotPos,
                                  const FreeFloatingVel& robotVel,
                                  const LinkNetExternalWrenches & linkExtWrenches,
                                  const JointDOFsDoubleArray & jointTorques,
                                        ArticulatedBodyAlgorithmInternalBuffers & buffers,
                                        FreeFloatingAcc & robotAcc);

    /**
     * \ingroup iDynTreeModel
     *
     *
     * @brief Compute the inverse dynamics of the model as linear function of the inertial parameters.
     *
     * This function computes the matrix that multiplied by the vector of inertial parameters of a model (see iDynTree::Model::getInertialParameters)
     * returns the inverse dynamics generalized torques. In particular it is consistent with the result of the iDynTree::RNEADynamicPhase function, i.e.
     * the first six rows of the regressor correspond to the sum of all external force/torques acting on the robot, expressed in the origin
     * and with the orientation of the specified referenceFrame, as defined by the referenceFrame_H_link argument.
     *
     *
     * @note The regressor only computes the inverse dynamics generalized torques assuming that the external forces are equal to zero,
     *       as the contribution of the external forces to the inverse dynamics is indipendent from inertial parameters.
     *
     * @param[in] model The model used for the computation.
     * @param[in] traversal The traversal used for the computation, it defines the used base link.
     * @param[in] referenceFrame_H_link Position of  each link w.r.t. to given frame D (tipically an inertial frame A, the base frame B or the mixed frame B[A]). For each link \f$L\f$, the corresponding transform is \f${}^D H_L\f$.
     * @param[in] linksVel Vector of left-trivialized velocities for each link of the model (for each link \f$L\f$, the corresponding velocity is \f${}^L \mathrm{v}_{A, L}\f$).
     * @param[in] linksProperAcc Vector of left-trivialized proper acceleration for each link of the model
     *                           (for each link \f$L\f$, the corresponding proper acceleration is \f${}^L \dot{\mathrm{v}}_{A, L} - \begin{bmatrix} {}^L R_A {}^A g \\ 0_{3\times1} \end{bmatrix} \f$), where \f$ {}^A g \in \mathbb{R}^3 \f$ is the gravity acceleration expressed in an inertial frame \f$A\f$ .
     * @param[out] baseForceAndJointTorquesRegressor The (6+model.getNrOfDOFs() X 10*model.getNrOfLinks()) inverse dynamics regressor.
     *
     */
    bool InverseDynamicsInertialParametersRegressor(const iDynTree::Model & model,
                                                    const iDynTree::Traversal & traversal,
                                                    const iDynTree::LinkPositions& referenceFrame_H_link,
                                                    const iDynTree::LinkVelArray & linksVel,
                                                    const iDynTree::LinkAccArray & linksAcc,
                                                          iDynTree::MatrixDynSize & baseForceAndJointTorquesRegressor);



}


#endif

