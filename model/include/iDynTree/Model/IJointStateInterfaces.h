/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_I_JOINT_STATE_INTERFACES_H
#define IDYNTREE_I_JOINT_STATE_INTERFACES_H

namespace iDynTree
{
    class IRawVector;

    /**
     * Interface (i.e. abstract class) exposed by class that
     * contain the position of a joint.
     *
     * \ingroup iDynTreeCore
     */
    class IJointPos
    {
    public:
        /**
         * Denstructor
         *
         */
        virtual ~IJointPos() = 0;

        /**
         * Accessor to the joint positions.
         */
        virtual IRawVector& pos() = 0;

        /**
         * Const accessor to the joint positions.
         */
        virtual const IRawVector& pos() const = 0;

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
    };

    /**
     * Interface (i.e. abstract class) exposed by class that
     * contain the position and the velocity of a joint.
     *
     * \ingroup iDynTreeCore
     */
    class IJointPosVel : public IJointPos
    {
    public:
        /**
         * Denstructor
         *
         */
        virtual ~IJointPosVel() = 0;

        /**
         * Accessor to the joint velocities.
         */
        virtual IRawVector& vel() = 0;

        /**
         * Const accessor to the joint velocities.
         */
        virtual const IRawVector& vel() const = 0;

        /**
         * Get the number of degrees of freedom of the joint.
         *
         * This should be a number between 0 (fixed joint) and 6 (free joint).
         *
         * @return the number of degrees of freedom of the joint.
         */
        virtual unsigned int getNrOfDOFs() const = 0;
    };


    /**
     * Interface (i.e. abstract class) exposed by class that
     * contain the position, velocity and acceleration of a joint.
     *
     * \ingroup iDynTreeModel
     */
    class IJointPosVelAcc : public IJointPosVel
    {
    public:
        /**
         * Denstructor
         *
         */
        virtual ~IJointPosVelAcc() = 0;

        /**
         * Accessor to the joint accelerations.
         */
        virtual IRawVector& acc() = 0;

        /**
         * Accessor to the joint accelerations.
         */
        virtual const IRawVector& acc() const = 0;
    };

    /**
     * Interface (i.e. abstract class) exposed by class that
     * contain the torque of a joint.
     *
     * \ingroup iDynTreeCore
     */
    class IJointTorque
    {
    public:
        /**
         * Denstructor
         *
         */
        virtual ~IJointTorque() = 0;

        /**
         * Accessor to the joint torques.
         */
        virtual IRawVector& torque() = 0;
    };


}

#endif /* IDYNTREE_I_JOINT_H */