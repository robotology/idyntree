/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_REVOLUTE_JOINT_H
#define IDYNTREE_REVOLUTE_JOINT_H

#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/SpatialMotionVector.h>

#include <iDynTree/Core/Axis.h>
#include <iDynTree/Model/Indeces.h>
#include <iDynTree/Model/MovableJointImpl.h>

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
        RevoluteJoint(const LinkIndex link1, const LinkIndex link2,
                      const Transform& link1_X_link2, const Axis& _rotation_axis_wrt_link1);

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

        // Set the revolute axis expressed in link1
        virtual void setAxis(const Axis& revoluteAxis_wrt_link1);

        // Documentation inherited
        virtual LinkIndex getFirstAttachedLink() const;

        // Documentation inherited
        virtual LinkIndex getSecondAttachedLink() const;

        /**
         * Get the revolute axis of the robot, expressed in linkA frame.
         *
         * @param linkA the link frame (one of the two at which the link is attached)
         *              in which the returned axis is expressed.
         *
         */
        virtual Axis getAxis(const LinkIndex linkA) const;

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

    };
}

#endif /* IDYNTREE_FIXED_JOINT_H */