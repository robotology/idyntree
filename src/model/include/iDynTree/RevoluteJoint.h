// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_REVOLUTE_JOINT_H
#define IDYNTREE_REVOLUTE_JOINT_H

#include <iDynTree/Transform.h>
#include <iDynTree/SpatialMotionVector.h>

#include <iDynTree/Axis.h>
#include <iDynTree/Indices.h>
#include <iDynTree/MovableJointImpl.h>

namespace iDynTree
{
    /**
     * Class representing a revolute joint, i.e. a joint that
     * constraint two links to move only around a common axis.
     *
     * \ingroup iDynTreeModel
     */
    class RevoluteJoint : public MovableJointImpl1
    {
    private:
        // Structure attributes
        LinkIndex link1;
        LinkIndex link2;
        Transform link1_X_link2_at_rest;
        Axis rotation_axis_wrt_link1;

        // Limits
        void disablePosLimits();
        bool m_hasPosLimits;
        double m_minPos;
        double m_maxPos;

        // Dynamic parameters
        void resetJointDynamics();
        JointDynamicsType m_joint_dynamics_type;
        double m_damping;
        double m_static_friction;

        // Cache attributes
        mutable double q_previous;
        mutable Transform link1_X_link2;
        mutable Transform link2_X_link1;
        mutable SpatialMotionVector S_link1_link2;
        mutable SpatialMotionVector S_link2_link1;

        void updateBuffers(const double new_q) const;
        void resetBuffers(const double new_q) const;
        void resetAxisBuffers() const;

    public:
        /**
         * Constructor
         */
        RevoluteJoint();

        IDYNTREE_DEPRECATED_WITH_MSG("Please use the setter methods to specify the parameters of the joint")
        RevoluteJoint(const LinkIndex link1, const LinkIndex link2,
                       const Transform& link1_X_link2, const Axis& _rotation_axis_wrt_link1);

        /**
         * Constructor in which the LinkIndex to which the joint is attached are not specified.
         * This constructor is tipically used together with the Model::addJoint or
         * Model::addJointAndLink methods, in which the links to which the joint is attached are
         * specified by the other arguments of the method.
         */
        RevoluteJoint(const Transform& link1_X_link2, const Axis& _rotation_axis_wrt_link1);

        /**
         * Copy constructor
         */
        RevoluteJoint(const RevoluteJoint& other);

        /**
         * Destructor
         */
        virtual ~RevoluteJoint();

        // Documentation inherited
        virtual IJoint * clone() const;

        // Documentation inherited
        virtual void setAttachedLinks(const LinkIndex link1, const LinkIndex link2);

        // Documentation inherited
        virtual void setRestTransform(const Transform& link1_X_link2);

        /**
         * Set the revolute axis of the joint, expressed in specified link frame, that is considered the "child"
         * frame regarding the sign of the axis.
         *
         * See getAxis method for more information.
         *
         * @warning This method should be called after a valid restTransform between link1 and link2 has been
         *          set by calling the setRestTransform method.
         */
        virtual void setAxis(const Axis& revoluteAxis,
                             const LinkIndex child,
                             const LinkIndex parent=LINK_INVALID_INDEX);

        // Set the revolute axis expressed in link1
        IDYNTREE_DEPRECATED_WITH_MSG("Please use the setAxis method in which the link considered \"child\" is explicitly specified")
        virtual void setAxis(const Axis& revoluteAxis_wrt_link1);

        // Documentation inherited
        virtual LinkIndex getFirstAttachedLink() const;

        // Documentation inherited
        virtual LinkIndex getSecondAttachedLink() const;

        /**
         * Get the revolute axis of the robot, expressed in child frame.
         *
         * @param child the link frame (one of the two at which the link is attached)
         *              in which the returned axis is expressed. Furthermore, the
         *              axis direction depends on the assumption that this frame is
         *              considered the "child" in the relationship.
         *
         * See
         *
         * Seth, A., Sherman, M., Eastman, P., & Delp, S. (2010).
         * Minimal formulation of joint motion for biomechanisms.
         * Nonlinear Dynamics, 62(1), 291-303.
         * https://nmbl.stanford.edu/publications/pdf/Seth2010.pdf
         * Section 2.4
         *
         * and
         *
         * "Modelling, Estimation and Identification of Humanoid Robots Dynamics"
         * Traversaro - Section 3.2
         * https://traversaro.github.io/preprints/traversaro-phd-thesis.pdf
         *
         * for more details.
         */
        virtual Axis getAxis(const LinkIndex child,
                             const LinkIndex parent=LINK_INVALID_INDEX) const;

        // Documentation inherited
        virtual Transform getRestTransform(const LinkIndex child,
                                           const LinkIndex parent) const;


        // Documentation inherited
        virtual const Transform & getTransform(const VectorDynSize & jntPos,
                                               const LinkIndex child,
                                               const LinkIndex parent) const;

        // Documentation inherited
        TransformDerivative getTransformDerivative(const VectorDynSize & jntPos,
                                                   const LinkIndex child,
                                                   const LinkIndex parent,
                                                   const int posCoord_i) const;

        // Documentation inherited
        virtual bool getPositionDerivativeVelocityJacobian(const iDynTree::Span<const double> jntPos,
                                                           MatrixView<double>& positionDerivative_J_velocity) const;

        // Documentation inherited
        virtual bool setJointPosCoordsToRest(iDynTree::Span<double> jntPos) const;

        // Documentation inherited
        virtual bool normalizeJointPosCoords(iDynTree::Span<double> jntPos) const;

        // Documentation inherited
        virtual SpatialMotionVector getMotionSubspaceVector(int dof_i,
                                                            const LinkIndex child,
                                                            const LinkIndex parent=LINK_INVALID_INDEX) const;

         // Documentation inherited
        virtual void computeChildPosVelAcc(const VectorDynSize & jntPos,
                                           const VectorDynSize & jntVel,
                                           const VectorDynSize & jntAcc,
                                           LinkPositions & linkPositions,
                                           LinkVelArray & linkVels,
                                           LinkAccArray & linkAccs,
                                           const LinkIndex child, const LinkIndex parent) const;

        // Documentation inherited
        virtual void computeChildVel(const VectorDynSize & jntPos,
                                     const VectorDynSize & jntVel,
                                     LinkVelArray & linkVels,
                                     const LinkIndex child, const LinkIndex parent) const;

        // Documentation inherited
        virtual void computeChildVelAcc(const VectorDynSize & jntPos,
                                        const VectorDynSize & jntVel,
                                        const VectorDynSize & jntAcc,
                                        LinkVelArray & linkVels,
                                        LinkAccArray & linkAccs,
                                        const LinkIndex child, const LinkIndex parent) const;

        // Documentation inherited
        virtual void computeChildAcc(const VectorDynSize & jntPos,
                                     const VectorDynSize & jntVel,
                                     const LinkVelArray & linkVels,
                                     const VectorDynSize & jntAcc,
                                     LinkAccArray & linkAccs,
                                     const LinkIndex child,
                                     const LinkIndex parent) const;

        // Documentation inherited
        virtual void computeChildBiasAcc(const VectorDynSize & jntPos,
                                         const VectorDynSize & jntVel,
                                         const LinkVelArray & linkVels,
                                         LinkAccArray & linkBiasAccs,
                                         const LinkIndex child,
                                         const LinkIndex parent) const;

        // Documentation inherited
        virtual void computeJointTorque(const VectorDynSize & jntPos, const Wrench & internalWrench,
                                        const LinkIndex linkThatAppliesWrench, const LinkIndex linkOnWhichWrenchIsApplied,
                                        VectorDynSize & jntTorques) const;

        // LIMITS METHODS
        virtual bool hasPosLimits() const;
        virtual bool enablePosLimits(const bool enable);
        virtual bool getPosLimits(const size_t _index, double & min, double & max) const;
        virtual double getMinPosLimit(const size_t _index) const;
        virtual double getMaxPosLimit(const size_t _index) const;
        virtual bool setPosLimits(const size_t _index, double min, double max);

        // DYNAMICS METHODS
        virtual JointDynamicsType getJointDynamicsType() const;
        virtual bool setJointDynamicsType(const JointDynamicsType enable);
        virtual double getDamping(const size_t _index) const;
        virtual double getStaticFriction(const size_t _index) const;
        virtual bool setDamping(const size_t _index, double damping);
        virtual bool setStaticFriction(const size_t _index, double staticFriction);
    };
}

#endif
