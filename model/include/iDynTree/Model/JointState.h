/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_JOINT_STATE_H
#define IDYNTREE_JOINT_STATE_H

#include <iDynTree/Core/VectorFixSize.h>

#include <iDynTree/Model/IJointStateInterfaces.h>

namespace iDynTree
{

    template <int nrOfPosCoords>
    class JointPos: public IJointPos
    {
    private:
        VectorFixSize<nrOfPosCoords> m_pos;

    public:
        // Documentation inherited
        IRawVector & pos();

        /**
          * Denstructor
          *
          */
        virtual ~JointPos();
    };

    /**
     * Interface (i.e. abstract class) exposed by class that
     * contain the position and the velocity of a joint.
     *
     * \ingroup iDynTreeCore
     */
    template <int nrOfPosCoords, int nrOfDOFs>
    class JointPosVel : public IJointPosVel, public JointPos<nrOfPosCoords>
    {
    private:
        VectorFixSize<nrOfDOFs> m_vel;

    public:
        // Documentation inherited
        IRawVector & vel();

        /**
         * Denstructor
         *
         */
        virtual ~JointPosVel();

        /**
         * Get the number of degrees of freedom of the joint.
         *
         * This should be a number between 0 (fixed joint) and 6 (free joint).
         *
         * @return the number of degrees of freedom of the joint.
         */
        virtual unsigned int getNrOfDOFs() const;
    };


    /**
     * Interface (i.e. abstract class) exposed by class that
     * contain the position, velocity and acceleration of a joint.
     *
     * \ingroup iDynTreeCore
     */
    template <int nrOfPosCoords, int nrOfDOFs>
    class JointPosVelAcc : public IJointPosVelAcc,
                           public JointPosVel<nrOfPosCoords,nrOfDOFs>
    {
    private:
        VectorFixSize<nrOfDOFs> m_acc;

    public:
        // Documentation inherited
        IRawVector & acc();

        /**
         * Denstructor
         *
         */
        virtual ~JointPosVelAcc();
    };

    IRawVector& JointPos::pos()
    {
        return this->m_pos;
    }

    // JointPos implementation
    JointPos::~JointPos()
    {

    }

    IRawVector& JointPosVel::vel()
    {
        return this->m_vel;
    }

    unsigned int JointPosVel::getNrOfDOFs() const
    {
        return nrOfDOFs;
    }

    JointPosVel::~JointPosVel()
    {

    }

    IRawVector& JointPosVelAcc::acc()
    {
        return this->m_acc;
    }

    JointPosVelAcc::~JointPosVelAcc()
    {

    }

}

#endif /* IDYNTREE_JOINT_STATE_H */