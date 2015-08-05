/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_I_JOINT_H
#define IDYNTREE_I_JOINT_H

namespace iDynTree
{
    /**
     * Interface (i.e. abstract class) exposed by classes that implement a Joint. 
     * A Joint is the basic representation of the motion allowed between two links. 
     * 
     * This interface is mean to be used by kinematics and dynamics algorithm to 
     * query informations related to a joint and the relations (relative position, 
     * relative twist, relative acceleration) that it imposes to the connected links.
     * 
     * The design of this class is heavily inspired by the Simbody implementation of joints, 
     * as described in this article: 
     * 
     * Seth, Ajay, et al. "Minimal formulation of joint motion for biomechanisms." 
     * Nonlinear dynamics 62.1-2 (2010): 291-303.
     * 
     * Other sources of inspiration are RBDL, Dart and Featherstone book.
     * 
     * With respect to all this implementation we model the joints as undirected quantities, 
     * i.e. as object in which information can be queryied in symmetric way with respect to the
     * attached links. This mean there is no parent and child link, but the joint is attached
     * to two link, and the interface is agnostic with respect to which link the code considers
     * as "parent" or "child".
     *
     * \ingroup iDynTreeCore
     */
    template<unsigned int nrOfPosCoords, unsigned int nrOfDOFs>
    class IJoint
    {
    public:
        /**
         * Denstructor
         *
         */
        virtual ~IJoint() = 0;

        /**
         * Get the number of coordinates used to represent 
         * the position of the joint.
         *
         * For joints whose configuration is in R^n, 
         * the number of position coordinates should 
         * match the number of degrees of freedom of the joint.
         *
         * @return the number of position coordinates.
         */
        virtual unsigned int getNrOfPosCoords() const;

        /**
         * Get the number of degrees of freedom of the joint.
         *
         * This should be a number between 0 (fixed joint) and 6 (free joint).
         *
         * @return the number of degrees of freedom of the joint.
         */
        virtual unsigned int getNrOfDOFs() const;

        /**
         * Get the transform between the linkB and the linkA, such that:
         * p_linkA = linkA_H_linkB*p_linkB,
         * where p_linkA is a quantity expressed in the linkA frame,
         * and   p_linkB is a quantity expressed in the linkB frame.
         */
        virtual Transform getTransform(const JointPos<nrOfPosCoords> & state, const int p_linkA, const int p_linkB) = 0;

        /**
         * Get the motion subspace matrix, i.e. the matrix that
         * maps the joint velocity to the relative twist between the two
         * links. 
         * In particular the motion subspace matrix is the S matrix such that
         * v_linkA = S_{linkA,linkB}*dq + linkA_X_linkB*v_linkB
         * where dq is the joint velocity.
         *
         * @return the motion subspace matrix
         *
         * \note The motion subspace matrix is also known in literature as hinge matrix,
         *       hinge  map matrix, joint map matrix  or joint  motion map matrix.
         */
        virtual MatrixFixSize<6, nrOfDOFs> getMotionSubspace(const JointPos<nrOfPosCoords> & state, const int linkA, const int linkB) const = 0;

        /**
         * Get the motion subspace matrix derivatives, i.e. the matrix that
         * maps the joint velocities to the relative spatial acceleration between the two
         * links.
         * In particular the motion subspace matrix derivative is the dS matrix such that
         * a_linkA = S_{linkA,linkB}*ddq + dS_{linkA,linkB}*dq + linkA_X_linkB*a_linkB
         * where ddq is the joint acceleration and dq is the joint velocity.
         *
         * @return the motion subspace matrix
         *
         * \note The motion subspace matrix is also known in literature as hinge matrix,
         *       hinge  map matrix, joint map matrix  or joint  motion map matrix.
         */
        virtual MatrixFixSize<6, nrOfDOFs> getMotionSubspaceDerivative(const JointPosVel<nrOfPosCoords,nrOfDOFs> & state, const int linkA, const int linkB) const = 0;

        /**
         * Get the kinematic coupling matrix, i.e. the matrix that maps 
         * the joint velocities to the derivative of the joint coordinates.
         * 
         * \note For the joint whose configuration is in R^n, the kinematic coupling matrix
         *  is equal to the identity.

         * @return the nrOfPosCoords times nrOfDOFs kinematic coupling matrix
         */
        virtual MatrixFixSize<nrOfPosCoords, nrOfDOFs> getKinematicCouplingMatrix(const JointPos<nrOfPosCoords> & state) = 0;

        /**
         * Compute the position, velocity and acceleration of linkA, 
         * given the position, velocty and acceleration of linkB and
         * the joint position, velocity and acceleration.
         * 
         * @return the linkA position, twist and spatial acceleration.
         */
        virtual LinkPosVelAcc computePosVelAcc(const JointPosVelAcc & state, const LinkPosVelAcc & linkBstate, const int linkA, const int linkB);

        /**
         * Compute the internal torque of joint, given the internal wrench that the linkThatAppliesWrench applies 
         * on the linkOnWhichWrenchIsApplied, expressed in the link frame of the linkOnWhichWrenchIsApplied.
         */
        virtual void computeJointTorque(const Wrench & internalWrench, IJointTorque & outputTorque, int linkThatAppliesWrench, int linkOnWhichWrenchIsApplied);
    };

    template<unsigned int nrOfPosCoords, unsigned int nrOfDOFs>
    virtual unsigned int IJoint<nrOfPosCoords,nrOfDOFs>::getNrOfPosCoords() const
    {
        return nrOfPosCoords;
    }

    template<unsigned int nrOfPosCoords, unsigned int nrOfDOFs>
    virtual unsigned int IJoint<nrOfPosCoords,nrOfDOFs>::getNrOfDOFs() const
    {
        return nrOfDOFs;
    }

}

#endif /* IDYNTREE_MATRIX_DYN_SIZE_H */