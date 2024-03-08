// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_I_JOINT_H
#define IDYNTREE_I_JOINT_H

#include <iDynTree/Indices.h>

namespace iDynTree
{
    class LinkPositions;
    class LinkVelArray;
    class LinkAccArray;
    class Transform;
    class TransformDerivative;
    class Wrench;
    class Twist;
    class VectorDynSize;
    class SpatialMotionVector;

    enum JointDynamicsType
    {
        /**
         * NoDynamics: No joint dynamics is assumed for the joint.
         * This joint dynamics type does not consider any parameter.
         */
        NoJointDynamics = 0,

        /**
         * URDFJointDynamics: Dynamics described by the URDF 1.0 specification.
         *
         * This joint dynamics type consider the following parameters:
         *  * `Damping`
         *  * `StaticFriction`
         */
        URDFJointDynamics = 1
    };

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
     * \ingroup iDynTreeModel
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
         * Get the transform between the link parent and the link child at joint position 0
         * (or at the identity configuration element for complex joints).
         * Such that:
         * p_child = child_H_parent*p_parent
         * where p_child is a quantity expressed in the child frame,
         * and   p_parent is a quantity expressed in the child frame.
         */
        virtual Transform getRestTransform(const LinkIndex child,
                                           const LinkIndex parent) const = 0;

        /**
         * Get the transform between the parent and the child, such that:
         * p_child = child_H_parent*p_parent,
         * where p_child is a quantity expressed in the child frame,
         * and   p_parent is a quantity expressed in the parent frame.
         */
        virtual const Transform & getTransform(const VectorDynSize & jntPos,
                                               const LinkIndex child,
                                               const LinkIndex parent) const = 0;

        /**
         * Get the derivative of the transform with
         * respect to a position coordinate.
         *
         * In particular, if the selected position coordinate is \f$q\f$, return the derivative:
         * \f[
         * \frac{\partial {}^\texttt{child} H_\texttt{parent} }{\partial q}
         * \f]
         *
         * If posCoord_i is not >= 0 and < getNrOfPosCoords(), the returned value is undefined.
         *
         */
        virtual TransformDerivative getTransformDerivative(const VectorDynSize & jntPos,
                                                           const LinkIndex child,
                                                           const LinkIndex parent,
                                                           const int posCoord_i) const = 0;

        /**
         * Get the motion subspace vector corresponding to the i-th
         * dof of the joint, i.e. the i-th column of  the motion subspace matrix.
         * The motion subspace matrix is the matrix that
         * maps the joint velocity to the relative twist between the two
         * links.
         *
         * In particular the motion subspace vector of the i-th dof is the S
         * vector  such that
         * v_child = S_{child,parent}*dq_i + child_X_parent*v_parent
         * if the velocities associated to all other DOFs of the joint
         * are considered zero, where v_child and v_parent are the left-trivialized
         * (body) velocities of the link child and parent.
         *
         * See
         * "Modelling, Estimation and Identification of Humanoid Robots Dynamics"
         * Silvio Traversaro - Section 3.2
         * https://traversaro.github.io/preprints/traversaro-phd-thesis.pdf
         * for more details.
         *
         * @return the motion subspace vector.
         *
         * If dof_i is not >= 0 and < getNrOfDOFs(), the returned value is undefined.
         *
         * \note The motion subspace matrix is also known in literature as hinge matrix,
         *       hinge  map matrix, joint map matrix  or joint  motion map matrix.
         */
        virtual SpatialMotionVector getMotionSubspaceVector(int dof_i,
                                                            const LinkIndex child,
                                                            const LinkIndex parent) const = 0;

        /**
         * Compute the position, velocity and acceleration of link child,
         * given the position, velocty and acceleration of link parent and
         * the joint position, velocity and acceleration.
         *
         * The position, velocity and acceleration of link child
         * are directly saved in the linkPositions, linkVels and linkAccs arguments.
         *
         */
        virtual void computeChildPosVelAcc(const VectorDynSize & jntPos,
                                           const VectorDynSize & jntVel,
                                           const VectorDynSize & jntAcc,
                                           LinkPositions & linkPositions,
                                           LinkVelArray & linkVels,
                                           LinkAccArray & linkAccs,
                                           const LinkIndex child, const LinkIndex parent) const = 0;

        /**
         * Compute the velocity and acceleration of child,
         * given the velocity and acceleration of parent and
         * the joint position, velocity and acceleration.
         *
         */
        virtual void computeChildVelAcc(const VectorDynSize & jntPos,
                                        const VectorDynSize & jntVel,
                                        const VectorDynSize & jntAcc,
                                        LinkVelArray & linkVels,
                                        LinkAccArray & linkAccs,
                                        const LinkIndex child, const LinkIndex parent) const = 0;

        /**
         * Compute the velocity of child,
         * given the velocity of parent  and
         * the joint position, velocity.
         */
        virtual void computeChildVel(const VectorDynSize & jntPos,
                                     const VectorDynSize & jntVel,
                                     LinkVelArray & linkVels,
                                     const LinkIndex child,
                                     const LinkIndex parent) const = 0;

        /**
         * Compute the (body-fixed) acceleration of a child link
         * given the (body-fixed) acceleration of the parent
         */
        virtual void computeChildAcc(const VectorDynSize & jntPos,
                                     const VectorDynSize & jntVel,
                                     const LinkVelArray & linkVels,
                                     const VectorDynSize & jntAcc,
                                           LinkAccArray & linkAccs,
                                     const LinkIndex child,
                                     const LinkIndex parent) const = 0;

        /**
         * Compute the (body-fixed) bias acceleration of a child link
         * given the (body-fixed) bias acceleration of the parent
         */
        virtual void computeChildBiasAcc(const VectorDynSize & jntPos,
                                         const VectorDynSize & jntVel,
                                         const LinkVelArray & linkVels,
                                               LinkAccArray & linkBiasAccs,
                                         const LinkIndex child,
                                         const LinkIndex parent) const = 0;

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
        virtual void computeJointTorque(const VectorDynSize & jntPos,
                                        const Wrench & internalWrench,
                                        const LinkIndex linkThatAppliesWrench,
                                        const LinkIndex linkOnWhichWrenchIsApplied,
                                        VectorDynSize & jntTorques) const = 0;

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

        /**
         * @name Limit handling methods.
         *  Methods for handling physical limits of joints.
         *
         *  The model used for limits is rather simple: a joint can have limits (being bounded)
         *  or not.
         *
         *  In the current version the limits are supported only for simple
         *  joints in which the velocity is the derivative of the position coordinate,
         *  and then getNrOfPosCoords() is equal to getNrOfDOFs() .
         *  The limits for such joints are specified by two constant vectors of dimension getNrOfDOFs(),
         *  the vector of minimum positions and the vector of maximum positions.
         */
        ///@{
        /**
         * Method to check if the joint has limits.
         *
         * @return true if the joints has limits
         */
        virtual bool hasPosLimits() const = 0;

        /**
         * Method to set if the joint has limits.
         *
         * @return true if everything went correctly, false otherwise
         *         (for example if the joint does not support joint position limits)
         */
        virtual bool enablePosLimits(const bool enable) = 0;

        /**
         * Get min and max position limits of the joint, for the _index dof.
         * @param[in] _index index of the dof for which the limit are obtained.
         * @return true if everything is correct, false otherwise.
         */
        virtual bool getPosLimits(const size_t _index, double & min, double & max) const = 0;

        /**
         * Get the min position limit of the joint, bindings-friendly version.
         */
        virtual double getMinPosLimit(const size_t _index) const = 0;

        /**
         * Get the max position limit of the joint, bindings-friendly version.
         */
        virtual double getMaxPosLimit(const size_t _index) const = 0;

        /**
         * Set the position limits for a dof the joint.
         *
         * @note This just sets the internal position limits of the joint.
         *       To set them as enabled, you need to call the enablePosLimits(true) method.
         */
        virtual bool setPosLimits(const size_t _index, double min, double max) = 0;

        /**
         * @name Joint dynamics methods.
         *
         *  Methods for handling representation of joint dynamics.
         *  The precise definition of "joint dynamics" is not precisely, as depending on the
         *  specific application the kind of joint dynamics model can be different, and in some
         *  case it may be even just instantaneous models (for example, when only the damping is considered).
         *
         *  For the type of joint dynamics supported, see the iDynTree::JointDynamicsType enum documentation.
         *
         *  The joint dynamics model are used in the following contexts:
         *   * In methods to serialize and deserialize URDF files
         *
         *  The joint dynamics are **not used at all** in classes to compute kinematics and dynamics quantities,
         *  such as iDynTree::KinDynComputations .
         */
        ///@{

        /**
         * Method to get the specific joint dynamics type used for the joint.
         * \note: It is assume that all the degrees of freedom of a joint share the same joint dynamics type.
         *
         * @return the specific joint dynamics type used for the joint.
         */
        virtual JointDynamicsType getJointDynamicsType() const = 0;

         /**
         * Method to get the specific joint dynamics type used for the joint.
         * \note: It is assume that all the degrees of freedom of a joint share the same joint dynamics type.
         *
         * @return true if everything went correctly, false otherwise
         */
        virtual bool setJointDynamicsType(const JointDynamicsType enable) = 0;

         /**
         * Set damping parameter of the joint, for the _index dof.
         * The damping coefficient is expressed in N∙s/m for a prismatic joint, N∙m∙s/rad for a revolute joint.
         *
         * This parameter is considered in the following joint dynamics types:
         * * `URDFJointDynamics`
         *
         * @param[in] _index index of the dof for which the dynamic parameters are obtained.
         * @return true if everything is correct, false otherwise.
         */
        virtual bool setDamping(const size_t _index, double damping) = 0;

         /**
         * Set static friction parameter of the joint, for the _index dof.
         * The static friction coefficient is expressed in N for a prismatic joint, N∙m for a revolute joint.
         *
         * This parameter is considered in the following joint dynamics types:
         * * `URDFJointDynamics`
         *
         * @param[in] _index index of the dof for which the dynamic parameters are obtained.
         * @return true if everything is correct, false otherwise.
         */
        virtual bool setStaticFriction(const size_t _index, double staticFriction) = 0;

        /**
         * Get the damping coefficient of the joint.
         * The unit is N∙s/m for a prismatic joint, N∙m∙s/rad for a revolute joint.
         *
         * This parameter is considered in the following joint dynamics types:
         * * `URDFJointDynamics`
         */
        virtual double getDamping(const size_t _index) const = 0;

        /**
         * Get the static friction coefficient of the joint.
         * The unit is N for a prismatic joint, N∙m for a revolute joint.
         *
         * This parameter is considered in the following joint dynamics types:
         * * `URDFJointDynamics`
         */
        virtual double getStaticFriction(const size_t _index) const = 0;


        ///@}
    };

    typedef IJoint * IJointPtr;
    typedef const IJoint * IJointConstPtr;


}

#endif /* IDYNTREE_I_JOINT_H */
