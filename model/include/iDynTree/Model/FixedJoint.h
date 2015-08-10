/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_FIXED_JOINT_H
#define IDYNTREE_FIXED_JOINT_H

#include <iDynTree/Core/Transform.h>

#include <iDynTree/Model/Indeces.h>
#include <iDynTree/Model/IJoint.h>

namespace iDynTree
{
    /**
     * Class representing a fixed joint, i.e. a joint that rigidly attaches two links.
     *
     * \ingroup iDynTreeCore
     */
    class FixedJoint : public IJoint
    {
    private:
        LinkIndex link1;
        LinkIndex link2;
        Transform link1_X_link2;

    public:
        /**
         * Constructor
         */
        FixedJoint(const LinkIndex link1, const LinkIndex link2,
                   const Transform& link1_X_link2);

        /**
         * Destructor
         */
        virtual ~FixedJoint();

        // Documentation inherited
        virtual unsigned int getNrOfPosCoords() const;

        // Documentation inherited
        virtual unsigned int getNrOfDOFs() const;

        // Documentation inherited
        virtual void setAttachedLinks(LinkIndex link1, LinkIndex link2);

        // Documentation inherited
        virtual void setRestTransform(const Transform& link1_X_link2);

        // Documentation inherited
        virtual Transform getTransform(const IJointPos & state, const LinkIndex p_linkA, const LinkIndex p_linkB);

         // Documentation inherited
        virtual LinkPosVelAcc computePosVelAcc(const IJointPosVelAcc & state, const LinkPosVelAcc & linkBstate,
                                               const LinkIndex linkA, const LinkIndex linkB);

        // Documentation inherited
        virtual LinkVelAcc computeVelAcc(const IJointPosVelAcc & state, const LinkVelAcc & linkBstate,
                                               const LinkIndex linkA, const LinkIndex linkB);

        // Documentation inherited
        virtual void computeJointTorque(const Wrench & internalWrench, IJointTorque & outputTorque,
                                        LinkIndex linkThatAppliesWrench, LinkIndex linkOnWhichWrenchIsApplied);
    };
}

#endif /* IDYNTREE_FIXED_JOINT_H */