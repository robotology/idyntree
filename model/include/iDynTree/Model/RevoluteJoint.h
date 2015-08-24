/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_REVOLUTE_JOINT_H
#define IDYNTREE_REVOLUTE_JOINT_H

#include <iDynTree/Core/Transform.h>

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
        LinkIndex link1;
        LinkIndex link2;
        Transform link1_X_link2_at_rest;
        Axis rotation_axis_wrt_link1;

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
        virtual Transform getTransform(const IJointPos & state, const LinkIndex p_linkA, const LinkIndex p_linkB) const;

         // Documentation inherited
        virtual LinkPosVelAcc computeLinkPosVelAcc(const IJointPosVelAcc & state, const LinkPosVelAcc & linkBstate,
                                               const LinkIndex linkA, const LinkIndex linkB) const;

        // Documentation inherited
        virtual LinkVelAcc computeLinkVelAcc(const IJointPosVelAcc & state, const LinkVelAcc & linkBstate,
                                               const LinkIndex linkA, const LinkIndex linkB) const;

        // Documentation inherited
        virtual void computeJointTorque(const IJointPos & state, const Wrench & internalWrench,
                                        const LinkIndex linkThatAppliesWrench, const LinkIndex linkOnWhichWrenchIsApplied,
                                        IJointTorque & outputTorque) const;
    };
}

#endif /* IDYNTREE_FIXED_JOINT_H */