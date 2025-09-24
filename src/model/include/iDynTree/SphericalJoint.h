// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_SPHERICAL_JOINT_H
#define IDYNTREE_SPHERICAL_JOINT_H

#include <iDynTree/Transform.h>
#include <iDynTree/SpatialMotionVector.h>

#include <iDynTree/Indices.h>
#include <iDynTree/MovableJointImpl.h>
#include <iDynTree/VectorFixSize.h>

namespace iDynTree
{
    /**
     * Class representing a spherical joint, i.e. a joint that
     * constrains two links to have the same position but allows
     * any relative orientation (3 rotational degrees of freedom).
     *
     * The joint state is parameterized using a unit quaternion
     * q = [qw, qx, qy, qz] (4 position coordinates), and the
     * angular velocity ω expressed in the first link frame
     * (3 velocity coordinates).
     *
     * Note: The quaternion follows the real/imaginary (w,x,y,z) convention
     * used by iDynTree, where w is the real part and (x,y,z) is the imaginary part.
     *
     * The 6D spatial vectors (velocity, acceleration, forces) follow the
     * linear/angular (linear first, angular second) convention used by iDynTree.
     *
     * \ingroup iDynTreeModel
     */
    class SphericalJoint : public MovableJointImpl<4,3>
    {
    private:
        // Structure attributes
        LinkIndex link1;
        LinkIndex link2;
        Transform link1_X_link2_at_rest;

        // Joint center offset (vector from link1 origin to joint center, expressed in link1 frame)
        Position m_center_wrt_link1;

        // Cache attributes
        mutable Vector4 q_previous;
        mutable Transform link1_X_link2;
        mutable Transform link2_X_link1;
        mutable SpatialMotionVector S_link1_link2[3];
        mutable SpatialMotionVector S_link2_link1[3];

        void updateBuffers(const Vector4& new_q) const;
        void resetBuffers(const Vector4& new_q) const;
        void resetAxisBuffers() const;

        // Helper methods
        void normalizeQuaternion(Vector4& q) const;

    public:
        /**
         * Constructor
         */
        SphericalJoint();

        /**
         * Constructor in which the LinkIndex to which the joint is attached are not specified.
         * This constructor is typically used together with the Model::addJoint or
         * Model::addJointAndLink methods, in which the links to which the joint is attached are
         * specified by the other arguments of the method.
         */
        SphericalJoint(const Transform& link1_X_link2);

        /**
         * Copy constructor
         */
        SphericalJoint(const SphericalJoint& other);

        /**
         * Destructor
         */
        virtual ~SphericalJoint();

        // Documentation inherited
        virtual IJoint * clone() const;

        // Documentation inherited
        virtual void setAttachedLinks(const LinkIndex link1, const LinkIndex link2);

        // Documentation inherited
        virtual void setRestTransform(const Transform& link1_X_link2);

        // Documentation inherited
        virtual LinkIndex getFirstAttachedLink() const;

        // Documentation inherited
        virtual LinkIndex getSecondAttachedLink() const;

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
                                     const LinkIndex child, const LinkIndex parent) const;

        // Documentation inherited
        virtual void computeChildBiasAcc(const VectorDynSize & jntPos,
                                         const VectorDynSize & jntVel,
                                         const LinkVelArray & linkVels,
                                         LinkAccArray & linkBiasAccs,
                                         const LinkIndex child, const LinkIndex parent) const;

        // Documentation inherited
        virtual void computeJointTorque(const VectorDynSize & jntPos,
                                        const Wrench & internalWrench,
                                        const LinkIndex linkThatAppliesWrench,
                                        const LinkIndex linkOnWhichWrenchIsApplied,
                                        VectorDynSize & jntTorques) const;

        // Documentation inherited
        virtual bool hasPosLimits() const;

        // Documentation inherited
        virtual bool enablePosLimits(const bool enable);

        // Documentation inherited
        virtual bool getPosLimits(const size_t _index, double & min, double & max) const;

        // Documentation inherited
        virtual double getMinPosLimit(const size_t _index) const;

        // Documentation inherited
        virtual double getMaxPosLimit(const size_t _index) const;

        // Documentation inherited
        virtual bool setPosLimits(const size_t _index, double min, double max);

        // Documentation inherited
        virtual JointDynamicsType getJointDynamicsType() const;

        // Documentation inherited
        virtual bool setJointDynamicsType(const JointDynamicsType enable);

        // Documentation inherited
        virtual bool setDamping(const size_t _index, double damping);

        // Documentation inherited
        virtual bool setStaticFriction(const size_t _index, double staticFriction);

        // Documentation inherited
        virtual double getDamping(const size_t _index) const;

        // Documentation inherited
        virtual double getStaticFriction(const size_t _index) const;

        // Documentation inherited
        virtual bool getPositionDerivativeVelocityJacobian(const iDynTree::Span<const double> jntPos,
                                                           iDynTree::MatrixView<double>& jac) const;

        // Documentation inherited
        virtual bool setJointPosCoordsToRest(iDynTree::Span<double> jntPos) const;

        // Documentation inherited
        virtual bool normalizeJointPosCoords(iDynTree::Span<double> jntPos) const;

        /**
         * Set the joint center relative to the given link.
         * @param link one of getFirstAttachedLink() or getSecondAttachedLink()
         * @param center vector from the link origin to the joint center, expressed in that link frame
         */
        void setJointCenter(const LinkIndex link, const Position& center);

        /**
         * Get the joint center relative to the given link.
         * @param link one of getFirstAttachedLink() or getSecondAttachedLink()
         * @return vector from the link origin to the joint center, expressed in that link frame
         */
        Position getJointCenter(const LinkIndex link) const;
    };
}

#endif
