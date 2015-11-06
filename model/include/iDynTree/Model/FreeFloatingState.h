/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_FREE_FLOATING_STATE_H
#define IDYNTREE_FREE_FLOATING_STATE_H

#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Twist.h>

#include <iDynTree/Model/Indeces.h>
#include <iDynTree/Model/JointState.h>
#include <iDynTree/Model/LinkState.h>

#include <vector>

namespace iDynTree
{
    class Model;

    /**
     * Class representing the position of a Free Floating robot.
     *
     * The position of a free floating robot is represented by the
     * transform of a base frame with respect to an inertial frame,
     * and a vector of joint positions.
     *
     */
    class FreeFloatingPos
    {
    private:
        Transform m_worldBasePos;

        VectorDynSize m_jointPos;

    public:
        // Documentation inherited
       FreeFloatingPos(const iDynTree::Model& model);

       /**
        * Resize the class to match the dimension of the joints contained in a Model.
        *
        * @param model the model from which to get the number and dimension of the joints.
        *
        * @warning This method wipes the joint positions if number and dimension of the joints
        *          of the model passed are different from the one already stored.
        *
        */
       void resize(const iDynTree::Model& model);

       /**
        * Get the base position.
        */
       Transform & worldBasePos();

       /**
        * Get the vector of joint positions.
        */
       IRawVector & jointPos();

       /**
        * Get the base position (const version).
        */
       const Transform & worldBasePos() const;

       /**
        * Get the vector of joint positions (const version).
        */
       const IRawVector & jointPos() const;

       /**
        * Get the dimension of the joint positions vector.
        */
       unsigned int getNrOfPosCoords() const;

        /**
          * Destructor
          */
        virtual ~FreeFloatingPos();
    };

     /**
     * Class representing the position, velocity and accelerations
     * of a Free Floating robot.
     *
     * The position of a free floating robot is represented by the
     * transform of a base frame with respect to an inertial frame,
     * and a vector of joint positions.
     *
     */
    class FreeFloatingPosVelAcc
    {
    private:
        LinkPosVelAcc m_basePosVelAcc;

        VectorDynSize m_jointPos;
        VectorDynSize m_jointVel;
        VectorDynSize m_jointAcc;

    public:
        // Documentation inherited
       FreeFloatingPosVelAcc(const iDynTree::Model& model);

       /**
        * Resize the class to match the dimension of the joints contained in a Model.
        *
        * @param model the model from which to get the number and dimension of the joints.
        *
        * @warning This method wipes the joint positions if number and dimension of the joints
        *          of the model passed are different from the one already stored.
        *
        */
       void resize(const iDynTree::Model& model);

       /**
        * Get the base position, velocity and accelerations.
        */
       LinkPosVelAcc & basePosVelAcc();

       /**
        * Get the joint position vector.
        */
       IRawVector & jointPos();

       /**
        * Get the joint velocities vector.
        */
       IRawVector & jointVel();

       /**
        * Get the joint accelerations vector.
        */
       IRawVector & jointAcc();

       /**
        * Get the base position (const version).
        */
       const LinkPosVelAcc & basePosVelAcc() const;

       /**
        * Get the joint positions vector (const version).
        */
       const IRawVector & jointPos() const;

       /**
        * Get the joint velocities vector (const version).
        */
       const IRawVector & jointVel() const;

       /**
        * Get the joint accelerations vector (const version).
        */
       const IRawVector & jointAcc() const;

       unsigned int getNrOfPosCoords() const;
       unsigned int getNrOfDOFs() const;

        /**
          * Destructor
          */
        virtual ~FreeFloatingPosVelAcc();
    };


    class FreeFloatingGeneralizedTorques
    {
    private:
        Wrench m_baseWrench;
        VectorDynSize m_jointTorques;

    public:
        // Documentation inherited
       FreeFloatingGeneralizedTorques(const iDynTree::Model& model);

       /**
        * Resize the class to match the dimension of the joints contained in a Model.
        *
        * @param model the model from which to get the number and dimension of the joints.
        *
        * @warning This method wipes the joint torques if number and dimension of the joints
        *          of the model passed are different from the one already stored.
        *
        */
       void resize(const iDynTree::Model& model);

       /**
        * Get the base wrench.
        */
       Wrench & baseWrench();

       /**
        * Get the joint torques vector.
        */
       IRawVector & jointTorques();

       /**
        * Get the base wrench (const version).
        */
       const Wrench & baseWrench() const;

       /**
        * Get the joint torques vector (const version).
        */
       const IRawVector & jointTorques() const;

       unsigned int getNrOfDOFs() const;

        /**
          * Destructor
          */
        virtual ~FreeFloatingGeneralizedTorques();
    };

}

#endif /* IDYNTREE_FREE_FLOATING_STATE_H */