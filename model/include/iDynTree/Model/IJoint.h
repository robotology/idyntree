/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_I_JOINT_H
#define IDYNTREE_I_JOINT_H

#include <iDynTree/Model/Indeces.h>

namespace iDynTree
{
    class LinkPosVelAcc;
    class LinkVelAcc;
    class Transform;
    class Wrench;
    class IRawVector;
    class SpatialMotionVector;

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
     * Other sources of inspiration are RBDL, DART and Featherstone book.
     *
     * With respect to all this implementation we model the joints as undirected quantities,
     * i.e. as object in which information can be queryied in symmetric way with respect to the
     * attached links. This mean there is no parent and child link, but the joint is attached
     * to two link, and the interface is agnostic with respect to which link the code considers
     * as "parent" or "child".
     *
     * \ingroup iDynTreeCore
     */
    class IJoint
    {
    public:
        /**
         * Denstructor
         *
         */
        virtual ~IJoint() = 0;

        /**
         * Clone the joint object.
         */
        virtual IJoint * clone() const = 0;

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
        virtual unsigned int getNrOfPosCoords() const = 0;

        /**
         * Get the number of degrees of freedom of the joint.
         *
         * This should be a number between 0 (fixed joint) and 6 (free joint).
         *
         * @return the number of degrees of freedom of the joint.
         */
        virtual unsigned int getNrOfDOFs() const = 0;


        /**
         * @name Methods to set joint
         * Methods to set joint informations (used when building a model)
         */
        //@{

        /**
         * Set the two links at which the joint is attached.
         * @param link1 is the first link
         * @param link2 is the second link
         */
        virtual void setAttachedLinks(const LinkIndex link1, const LinkIndex link2) = 0;

        /**
         * Set the transform between the link2 frame and link1 frame at joint position 0
         * (or at the identity configuration element for complex joints).
         *
         * The link1_T_link2 is transform that transforms a quantity
         * expressed in link2 frame in a quantity expressed in the link1
         * frame, when the joint is in the 0 position :
         * p_link1 = link1_T_link2*p_link2 .
         */
        virtual void setRestTransform(const Transform& link1_X_link2) = 0;

        //@}

        /**
         * Get the first link attached to the joint.
         */
        virtual LinkIndex getFirstAttachedLink() const = 0;

        /**
         * Get the second link attached to the joint.
         */
        virtual LinkIndex getSecondAttachedLink() const = 0;


        /**
         * Get the transform between the linkB and the linkA at joint position 0
         * (or at the identity configuration element for complex joints).
         * Such that:
         * p_linkA = linkA_H_linkB*p_linkB
         * where p_linkA is a quantity expressed in the linkA frame,
         * and   p_linkB is a quantity expressed in the linkB frame.
         */
        virtual Transform getRestTransform(const LinkIndex p_linkA,
                                           const LinkIndex p_linkB) const = 0;

        /**
         * Get the transform between the linkB and the linkA, such that:
         * p_linkA = linkA_H_linkB*p_linkB,
         * where p_linkA is a quantity expressed in the linkA frame,
         * and   p_linkB is a quantity expressed in the linkB frame.
         */
        virtual Transform getTransform(const IRawVector & jntPos,
                                       const LinkIndex p_linkA,
                                       const LinkIndex p_linkB) const = 0;

        /**
         * Get the motion subspace vector corresponding to the i-th
         * dof of the joint, i.e. the i-th column of  the motion subspace matrix.
         * The motion subspace matrix is the matrix that
         * maps the joint velocity to the relative twist between the two
         * links.
         *
         * In particular the motion subspace vector of the i-th dof is the S
         * vector  such that
         * v_linkA = S_{linkA,linkB}*dq_i + linkA_X_linkB*v_linkB
         * if the velocities associated to all other DOFs of the joint
         * are considered zero.
         *
         * @return the motion subspace vector.
         *
         * If dof_i is not >= 0 and < getNrOfDOFs(), the returned value is undefined.
         *
         * \note The motion subspace matrix is also known in literature as hinge matrix,
         *       hinge  map matrix, joint map matrix  or joint  motion map matrix.
         */
        virtual SpatialMotionVector getMotionSubspaceVector(int dof_i,
                                                            const LinkIndex p_linkA,
                                                            const LinkIndex p_linkB) const = 0;

        /**
         * Compute the position, velocity and acceleration of linkA,
         * given the position, velocty and acceleration of linkB and
         * the joint position, velocity and acceleration.
         *
         * @return the linkA position, twist and spatial acceleration.
         */
        virtual LinkPosVelAcc computeLinkPosVelAcc(const IRawVector & jntPos,
                                                   const IRawVector & jntVel,
                                                   const IRawVector & jntAcc,
                                                   const LinkPosVelAcc & linkBstate,
                                                   const LinkIndex linkA, const LinkIndex linkB) const = 0;

        /**
         * Compute the velocity and acceleration of linkA,
         * given the velocty and acceleration of linkB and
         * the joint position, velocity and acceleration.
         *
         * @return the linkA position, twist and spatial acceleration.
         */
        virtual LinkVelAcc computeLinkVelAcc(const IRawVector & jntPos,
                                             const IRawVector & jntVel,
                                             const IRawVector & jntAcc,
                                             const LinkVelAcc & linkBstate,
                                             const LinkIndex linkA, const LinkIndex linkB) const = 0;

        /**
         * Compute the internal torque of joint, given the internal wrench that the linkThatAppliesWrench applies
         * on the linkOnWhichWrenchIsApplied, expressed in the link frame of the linkOnWhichWrenchIsApplied.
         *
         * @param[in] jntPos vector of joint positions.
         * @param[in] internalWrench internal wrench that the linkThatAppliesWrench applies
         *                           on the linkOnWhichWrenchIsApplied, expressed in the link
         *                           frame of the linkOnWhichWrenchIsApplied
         * @param[in] linkThatAppliesWrench link index of the link that applies the considered internal wrench.
         * @param[in] linkOnWhichWrenchIsApplied link index of the link on which the considered internal wrench is applied.
         * @param[out] jntTorques vector of joint torques.
         */
        virtual void computeJointTorque(const IRawVector & jntPos,
                                        const Wrench & internalWrench,
                                        const LinkIndex linkThatAppliesWrench,
                                        const LinkIndex linkOnWhichWrenchIsApplied,
                                        IRawVector & jntTorques) const = 0;

        /**
         * Set the index of the joint in the Model Joint serialization.
         *
         */
        virtual void setIndex(JointIndex & _index) = 0;

        /**
         * Get the index of the joint in the model Joint serialization.
         */
        virtual JointIndex getIndex() const = 0;


        /**
         * Set the offset of the position coordinates of this
         * joint in the position coordiantes serialization of the model.
         */
        virtual void setPosCoordsOffset(const size_t _index) = 0;

        /**
         * Get the offset of the position coordinates of
         * this joint in the position coordiantes serialization of the model.
         */
        virtual size_t getPosCoordsOffset() const = 0;

        /**
         * Set the offset of the coordinates of this
         * joint in the velocity/acceleration coordiantes serialization of the model.
         */
        virtual void setDOFsOffset(const size_t _index) = 0;

        /**
         * Get the offset of the position coordinates of
         * joint in the velocity/acceleration coordiantes serialization of the model.
         */
        virtual size_t getDOFsOffset() const = 0;


    };

    typedef IJoint * IJointPtr;
    typedef const IJoint * IJointConstPtr;


}

#endif /* IDYNTREE_I_JOINT_H */