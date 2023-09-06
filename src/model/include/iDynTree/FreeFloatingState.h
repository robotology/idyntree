// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_FREE_FLOATING_STATE_H
#define IDYNTREE_FREE_FLOATING_STATE_H

#include <iDynTree/Transform.h>
#include <iDynTree/Twist.h>
#include <iDynTree/VectorDynSize.h>

#include <iDynTree/Indices.h>
#include <iDynTree/LinkState.h>
#include <iDynTree/JointState.h>

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
        JointPosDoubleArray m_jointPos;

    public:
        // Documentation inherited
       FreeFloatingPos();
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
       JointPosDoubleArray & jointPos();

       /**
        * Get the base position (const version).
        */
       const Transform & worldBasePos() const;

       /**
        * Get the vector of joint positions (const version).
        */
       const JointPosDoubleArray & jointPos() const;

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
        JointDOFsDoubleArray m_jointTorques;

    public:
        // Documentation inherited
       FreeFloatingGeneralizedTorques();
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
       JointDOFsDoubleArray & jointTorques();

       /**
        * Get the base wrench (const version).
        */
       const Wrench & baseWrench() const;

       /**
        * Get the joint torques vector (const version).
        */
       const JointDOFsDoubleArray & jointTorques() const;

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
        JointDOFsDoubleArray m_jointVel;

    public:
        // Documentation inherited
        FreeFloatingVel();
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
       JointDOFsDoubleArray & jointVel();

       /**
        * Get the base acceleration (const version).
        */
       const Twist & baseVel() const;

       /**
        * Get the vector of joint accelerations (const version).
        */
       const JointDOFsDoubleArray & jointVel() const;

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
        JointDOFsDoubleArray m_jointAcc;

    public:
        // Documentation inherited
        FreeFloatingAcc();
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
       JointDOFsDoubleArray & jointAcc();

       /**
        * Get the base acceleration (const version).
        */
       const SpatialAcc & baseAcc() const;

       /**
        * Get the vector of joint accelerations (const version).
        */
       const JointDOFsDoubleArray & jointAcc() const;

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

#endif
