/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_I_JOINT_H
#define IDYNTREE_I_JOINT_H

#include <iDynTree/Core/IVector.h>

#include <iDynTree/Model/IJoint.h>


namespace iDynTree
{
    // \todo TODO define a proper IJointTorque interface
    typedef IVector IJointTorque;

    /**
     * Base template for implementation of non-fixed joints.
     * A specific joint can be derived from an instantiation of this template.
     *
     * For more information on the assumption of the joint model, check the IJoint interface.
     *
     * \ingroup iDynTreeCore
     */
    template<unsigned int nrOfPosCoords, unsigned int nrOfDOFs>
    class MovableJointImpl : public IJoint
    {
    public:
        /**
         * Denstructor
         *
         */
        virtual ~MovableJointImpl() = 0;

        // Documentation inherited
        virtual unsigned int getNrOfPosCoords() const;

        // Documentation inherited
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
        virtual LinkPosVelAcc computePosVelAcc(const JointPosVelAcc & state, const LinkPosVelAcc & linkBstate,
                                               const int linkA, const int linkB);

        /**
         * Compute the internal torque of joint, given the internal wrench that the linkThatAppliesWrench applies
         * on the linkOnWhichWrenchIsApplied, expressed in the link frame of the linkOnWhichWrenchIsApplied.
         */
        virtual void computeJointTorque(const Wrench & internalWrench, IJointTorque & outputTorque,
                                        int linkThatAppliesWrench, int linkOnWhichWrenchIsApplied);
    };

    template<unsigned int nrOfPosCoords, unsigned int nrOfDOFs>
    virtual unsigned int MovableJointImpl<nrOfPosCoords,nrOfDOFs>::getNrOfPosCoords() const
    {
        return nrOfPosCoords;
    }

    template<unsigned int nrOfPosCoords, unsigned int nrOfDOFs>
    virtual unsigned int MovableJointImpl<nrOfPosCoords,nrOfDOFs>::getNrOfDOFs() const
    {
        return nrOfDOFs;
    }

    typedef MovableJointImpl<1,1> MovableJointImpl1;
    typedef MovableJointImpl<2,2> MovableJointImpl2;
    typedef MovableJointImpl<3,3> MovableJointImpl3;
    typedef MovableJointImpl<4,4> MovableJointImpl4;
    typedef MovableJointImpl<5,5> MovableJointImpl5;
    typedef MovableJointImpl<6,6> MovableJointImpl6;
}

#endif /* IDYNTREE_JOINT_IMPL_H */