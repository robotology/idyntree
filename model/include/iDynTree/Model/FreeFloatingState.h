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
#include <iDynTree/Core/VectorDynSize.h>

#include <iDynTree/Model/Indeces.h>
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


    class FreeFloatingVel
    {
    private:
        Twist m_baseVel;
        VectorDynSize m_jointVel;

    public:
        // Documentation inherited
       FreeFloatingVel(const iDynTree::Model& model);

       /**
        * Resize the class to match the number of internal DOFs contained in a Model.
        *
        * @param model the model from which to get the number and dimension of the joints.
        *
        * @warning This method wipes the joint velocities if number and dimension of the joints
        *          of the model passed are different from the one already stored.
        *
        */
       void resize(const iDynTree::Model& model);

       /**
        * Get the base acceleration.
        */
       Twist & baseVel();

       /**
        * Get the vector of joint accelerations.
        */
       IRawVector & jointVel();

       /**
        * Get the base acceleration (const version).
        */
       const Twist & baseVel() const;

       /**
        * Get the vector of joint accelerations (const version).
        */
       const IRawVector & jointVel() const;

       /**
        * Get the dimension of the joint accelerations vector.
        */
       unsigned int getNrOfDOFs() const;

        /**
          * Destructor
          */
        virtual ~FreeFloatingVel();
    };

    /**
     * Class representing the accelerations of a Free Floating robot.
     *
     */
    class FreeFloatingAcc
    {
    private:
        SpatialAcc m_baseAcc;
        VectorDynSize m_jointAcc;

    public:
        // Documentation inherited
       FreeFloatingAcc(const iDynTree::Model& model);

       /**
        * Resize the class to match the number of internal DOFs contained in a Model.
        *
        * @param model the model from which to get the number and dimension of the joints.
        *
        * @warning This method wipes the joint positions if number and dimension of the joints
        *          of the model passed are different from the one already stored.
        *
        */
       void resize(const iDynTree::Model& model);

       /**
        * Get the base acceleration.
        */
       SpatialAcc & baseAcc();

       /**
        * Get the vector of joint accelerations.
        */
       IRawVector & jointAcc();

       /**
        * Get the base acceleration (const version).
        */
       const SpatialAcc & baseAcc() const;

       /**
        * Get the vector of joint accelerations (const version).
        */
       const IRawVector & jointAcc() const;

       /**
        * Get the dimension of the joint accelerations vector.
        */
       unsigned int getNrOfDOFs() const;

        /**
          * Destructor
          */
        virtual ~FreeFloatingAcc();
    };

}

#endif /* IDYNTREE_FREE_FLOATING_STATE_H */