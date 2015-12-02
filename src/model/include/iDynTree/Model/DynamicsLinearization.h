/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_DYNAMICS_LINEARIZATION_H
#define IDYNTREE_DYNAMICS_LINEARIZATION_H

#include <iDynTree/Core/MatrixFixSize.h>
#include <iDynTree/Core/MatrixDynSize.h>

#include <iDynTree/Model/Dynamics.h>

namespace iDynTree
{
    /**
     * Structure containing the internal buffers used
     * by the ForwardDynamicsLinearization function.
     */
    struct ForwardDynamicsLinearizationInternalBuffers
    {
        ForwardDynamicsLinearizationInternalBuffers() {};

        /**
         * Call resize(model);
         */
        ForwardDynamicsLinearizationInternalBuffers(const Model & model);

        /**
         * Resize all the buffers to the right size given the model,
         * and reset all the buffers to 0.
         */
        void resize(const Model& model);

        /**
         * Buffers used for computing the ABA algorithm.
         */
        ArticulatedBodyAlgorithmInternalBuffers aba;

        /**
         * Buffers used for computing the derivative of ABA with respect
         * to the joint DOFs position.
         */
        std::vector<ArticulatedBodyAlgorithmInternalBuffers> dPos;

        /**
         * Buffers used to compute the derivative with respect to the base velocity.
         */
        LinkPositions linkPos;
        std::vector<Matrix6x6> dVb_c;
        std::vector<Matrix6x6> dVb_linkBiasWrench;
        std::vector<Matrix1x6> dVb_u;
        std::vector<Matrix6x6> dVb_linkBiasAcceleration;
        std::vector<Matrix6x6> dVb_linksAcceleration;

        /**
         * Buffer to store the derivative of
         * the product
         * \f[
         *   V_l \bar\cross^* M_l V_l
         * \f]
         *
         * with respect to V_l
         */
        std::vector<Matrix6x6> dVl_linkLocalBiasWrench;


        /**
         * Buffers used for computing the derivative of ABA with respect
         * to the joint DOFs velocities.
         */
        std::vector<ArticulatedBodyAlgorithmInternalBuffers> dVel;
    };

    class FreeFloatingStateLinearization : public MatrixDynSize
    {
    public:
        FreeFloatingStateLinearization();

        FreeFloatingStateLinearization(const Model & model);

        void resize(const Model & model);
    };

    /*
     * Compute the left-trivialized linearization,
     * as described in [fill with left-trivialized repot when available].
     *
     */
    bool ForwardDynamicsLinearization(const Model& model,
                                      const Traversal& traversal,
                                      const FreeFloatingPos& robotPos,
                                      const FreeFloatingVel& robotVel,
                                      const LinkExternalWrenches & linkExtWrenches,
                                      const JointDoubleArray & jointTorques,
                                            ForwardDynamicsLinearizationInternalBuffers bufs,
                                            FreeFloatingAcc & robotAcc,
                                            FreeFloatingStateLinearization & A);

}


#endif /* IDYNTREE_DYNAMICS_H */