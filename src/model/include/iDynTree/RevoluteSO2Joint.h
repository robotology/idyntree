// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_REVOLUTE_SO2_JOINT_H
#define IDYNTREE_REVOLUTE_SO2_JOINT_H

#include <iDynTree/Transform.h>
#include <iDynTree/SpatialMotionVector.h>

#include <iDynTree/Axis.h>
#include <iDynTree/Indices.h>
#include <iDynTree/MovableJointImpl.h>

namespace iDynTree
{
    /**
     * Class representing a revolute joint using SO(2) representation, i.e. a joint that
     * constrains two links to move only around a common axis.
     * 
     * Unlike the regular RevoluteJoint, this joint represents the angular
     * position using a complex number (with real and imaginary parts)
     * to avoid periodicity issues. Each angle corresponds to a unique
     * complex number on the unit circle.
     *
     * The joint has:
     * - 2 position coordinates: real and imaginary parts of the complex number
     * - 1 degree of freedom: the angular velocity
     * 
     * The actual angle is computed as atan2(imaginary_part, real_part).
     *
     * \ingroup iDynTreeModel
     */
    class RevoluteSO2Joint : public MovableJointImpl21
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
        mutable double q_real_previous;
        mutable double q_imag_previous;
        mutable Transform link1_X_link2;
        mutable Transform link2_X_link1;
        mutable SpatialMotionVector S_link1_link2;
        mutable SpatialMotionVector S_link2_link1;

        void updateBuffers(const double new_q_real, const double new_q_imag) const;
        void resetBuffers(const double new_q_real, const double new_q_imag) const;
        void resetAxisBuffers() const;

        /**
         * Convert complex coordinates to angle.
         * @param q_real real part of the complex number
         * @param q_imag imaginary part of the complex number
         * @return the angle in radians
         */
        double complexToAngle(const double q_real, const double q_imag) const;

        /**
         * Normalize the complex number to unit circle.
         * @param q_real reference to real part (will be modified)
         * @param q_imag reference to imaginary part (will be modified)
         */
        void normalizeComplex(double& q_real, double& q_imag) const;

    public:
        /**
         * Constructor
         */
        RevoluteSO2Joint();

        IDYNTREE_DEPRECATED_WITH_MSG("Please use the setter methods to specify the parameters of the joint")
        RevoluteSO2Joint(const LinkIndex link1, const LinkIndex link2,
                           const Transform& link1_X_link2, const Axis& _rotation_axis_wrt_link1);

        /**
         * Constructor in which the LinkIndex to which the joint is attached are not specified.
         * This constructor is typically used together with the Model::addJoint or
         * Model::addJointAndLink methods, in which the links to which the joint is attached are
         * specified by the other arguments of the method.
         */
        RevoluteSO2Joint(const Transform& link1_X_link2, const Axis& _rotation_axis_wrt_link1);

        /**
         * Copy constructor
         */
        RevoluteSO2Joint(const RevoluteSO2Joint& other);

        /**
         * Destructor
         */
        virtual ~RevoluteSO2Joint();

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

        /**
         * Get the current angle from the complex representation.
         * @param jntPos the joint positions vector
         * @return the angle in radians
         */
        double getCurrentAngle(const VectorDynSize & jntPos) const;

        /**
         * Set joint position from an angle.
         * @param jntPos the joint positions vector to modify
         * @param angle the angle in radians
         */
        void setJointPositionFromAngle(VectorDynSize & jntPos, double angle) const;
    };
}

#endif
