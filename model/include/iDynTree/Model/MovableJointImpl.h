/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_MOVABLE_JOINT_IMPL_H
#define IDYNTREE_MOVABLE_JOINT_IMPL_H

#include <iDynTree/Core/IVector.h>

#include <iDynTree/Model/IJoint.h>


namespace iDynTree
{
    /**
     * Base template for implementation of non-fixed joints.
     * A specific joint can be derived from an instantiation of this template.
     *
     * For more information on the assumption of the joint model, check the IJoint interface.
     *
     * \ingroup iDynTreeCore
     */
    template<unsigned int nrOfPosCoords, unsigned int nrOfDOFs>
    class MovableJointImpl: public IJoint
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
        //virtual MatrixFixSize<6, nrOfDOFs> getMotionSubspace(const JointPos<nrOfPosCoords> & state, const int linkA, const int linkB) const = 0;

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
        //virtual MatrixFixSize<6, nrOfDOFs> getMotionSubspaceDerivative(const JointPosVel<nrOfPosCoords,nrOfDOFs> & state, const int linkA, const int linkB) const = 0;

        /**
         * Get the kinematic coupling matrix, i.e. the matrix that maps
         * the joint velocities to the derivative of the joint coordinates.
         *
         * \note For the joint whose configuration is in R^n, the kinematic coupling matrix
         *  is equal to the identity.

         * @return the nrOfPosCoords times nrOfDOFs kinematic coupling matrix
         */
        //virtual MatrixFixSize<nrOfPosCoords, nrOfDOFs> getKinematicCouplingMatrix(const JointPos<nrOfPosCoords> & state) = 0;

    };

    template<unsigned int nrOfPosCoords, unsigned int nrOfDOFs>
    unsigned int MovableJointImpl<nrOfPosCoords,nrOfDOFs>::getNrOfPosCoords() const
    {
        return nrOfPosCoords;
    }

    template<unsigned int nrOfPosCoords, unsigned int nrOfDOFs>
    unsigned int MovableJointImpl<nrOfPosCoords,nrOfDOFs>::getNrOfDOFs() const
    {
        return nrOfDOFs;
    }

    template<unsigned int nrOfPosCoords, unsigned int nrOfDOFs>
    MovableJointImpl<nrOfPosCoords,nrOfDOFs>::~MovableJointImpl()
    {
    }

    typedef MovableJointImpl<1,1> MovableJointImpl1;
    typedef MovableJointImpl<2,2> MovableJointImpl2;
    typedef MovableJointImpl<3,3> MovableJointImpl3;
    typedef MovableJointImpl<4,4> MovableJointImpl4;
    typedef MovableJointImpl<5,5> MovableJointImpl5;
    typedef MovableJointImpl<6,6> MovableJointImpl6;
}

#endif /* IDYNTREE_JOINT_IMPL_H */