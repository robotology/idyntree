/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_LINK_STATE_H
#define IDYNTREE_LINK_STATE_H

#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/SpatialAcc.h>

#include <iDynTree/Model/Indeces.h>

#include <vector>

namespace iDynTree
{
    class Model;

    class LinkPos
    {
    private:
        Transform m_pos;

    public:
        // Documentation inherited
        Transform & pos();

        const Transform & pos() const;

        /**
          * Destructor
          */
        virtual ~LinkPos();
    };

    class LinkPositions
    {
    private:
        std::vector<LinkPos> m_linkPos;

    public:
        LinkPositions(unsigned int nrOfLinks = 0);
        LinkPositions(const iDynTree::Model & model);

        void resize(unsigned int nrOfLinks);
        void resize(const iDynTree::Model & model);

        LinkPos & linkPos(const LinkIndex link);
        const LinkPos & linkPos(const LinkIndex link) const;

        ~LinkPositions();
    };

    class LinkVelAcc
    {
    private:
        Twist      m_vel;
        SpatialAcc m_acc;

    public:
        Twist      & vel();
        SpatialAcc & acc();

        const Twist      & vel() const;
        const SpatialAcc & acc() const;

        /**
          * Destructor
          */
        virtual ~LinkVelAcc();
    };

    class LinkPosVelAcc
    {
    private:
        Transform  m_pos;
        Twist      m_vel;
        SpatialAcc m_acc;

    public:
        Transform & pos();
        Twist     & vel();
        SpatialAcc& acc();

        const Transform & pos() const;
        const Twist     & vel() const;
        const SpatialAcc& acc() const;

        /**
          * Destructor
          */
        virtual ~LinkPosVelAcc();
    };
}

#endif /* IDYNTREE_LINK_STATE_H */