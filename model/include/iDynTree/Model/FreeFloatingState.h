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

        /**
         * \todo TODO : check if the double indirection caused by
         *              this vector of pointers is a performance bottleneck.
         *              In that case substitute with a continuos array + offsets.
         *
         */
        std::vector<IJointPos *> m_jointPos;

        /**
         *
         */
        unsigned int nrOfPosCoords;

        /**
         * Clear the joint pos vector.
         */
        void clearJointPos();

        void buildJointPos(const Model& model);

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
        * Get the joint positions.
        */
       IJointPos & jointPos(const JointIndex jointIndex);

       /**
        * Get the base position (const version).
        */
       const Transform & worldBasePos() const;

       /**
        * Get the joint positions (const version).
        */
       const IJointPos & jointPos(const JointIndex jointIndex) const;

       /**
        *
        */
       unsigned int getNrOfPosCoords() const;

        /**
          * Destructor
          */
        virtual ~FreeFloatingPos();
    };

}

#endif /* IDYNTREE_FREE_FLOATING_STATE_H */