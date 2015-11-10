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
#include <iDynTree/Core/Wrench.h>

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

    class LinkPosVel
    {
    private:
        Transform  m_pos;
        Twist      m_vel;

    public:
        Transform & pos();
        Twist     & vel();

        const Transform & pos() const;
        const Twist     & vel() const;

        /**
          * Destructor
          */
        virtual ~LinkPosVel();
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

    class LinkVelAccArray
    {
    private:
        std::vector<LinkVelAcc> m_linkState;

    public:
        LinkVelAccArray(unsigned int nrOfLinks = 0);
        LinkVelAccArray(const iDynTree::Model & model);

        void resize(unsigned int nrOfLinks);
        void resize(const iDynTree::Model & model);

        LinkVelAcc & linkVelAcc(const LinkIndex link);
        const LinkVelAcc & linkVelAcc(const LinkIndex link) const;

        ~LinkVelAccArray();
    };

    class LinkPosVelAccArray
    {
    private:
        std::vector<LinkPosVelAcc> m_linkState;

    public:
        LinkPosVelAccArray(unsigned int nrOfLinks = 0);
        LinkPosVelAccArray(const iDynTree::Model & model);

        void resize(unsigned int nrOfLinks);
        void resize(const iDynTree::Model & model);

        LinkPosVelAcc & linkPosVelAcc(const LinkIndex link);
        const LinkPosVelAcc & linkPosVelAcc(const LinkIndex link) const;

        ~LinkPosVelAccArray();
    };

    class LinkWrenches
    {
    private:
        std::vector<iDynTree::Wrench> m_linkWrenches;

    public:
        LinkWrenches(unsigned int nrOfLinks = 0);
        LinkWrenches(const iDynTree::Model & model);

        void resize(unsigned int nrOfLinks);
        void resize(const iDynTree::Model & model);

        iDynTree::Wrench & operator()(const LinkIndex link);
        const iDynTree::Wrench & operator()(const LinkIndex link) const;

        ~LinkWrenches();
    };

    typedef LinkWrenches LinkExternalWrenches;
    typedef LinkWrenches LinkInternalWrenches;

    /**
     * Class for storing a vector of SpatialInertia objects , one for each link in a model.
     */
    class LinkInertias
    {
    private:
        std::vector<iDynTree::SpatialInertia> m_linkInertials;

    public:
        LinkInertias(unsigned int nrOfLinks = 0);
        LinkInertias(const iDynTree::Model & model);

        void resize(unsigned int nrOfLinks);
        void resize(const iDynTree::Model & model);

        iDynTree::SpatialInertia & operator()(const LinkIndex link);
        const iDynTree::SpatialInertia & operator()(const LinkIndex link) const;

        ~LinkInertias();
    };

    typedef LinkInertias LinkCompositeRigidBodyInertias;

    /**
     * Class for storing a vector of ArticulatedBodyInertias objects , one for each link in a model.
     */
    class LinkArticulatedBodyInertias
    {
    private:
        std::vector<iDynTree::ArticulatedBodyInertia> m_linkABIs;

    public:
        LinkArticulatedBodyInertias(unsigned int nrOfLinks = 0);
        LinkArticulatedBodyInertias(const iDynTree::Model & model);

        void resize(unsigned int nrOfLinks);
        void resize(const iDynTree::Model & model);

        iDynTree::ArticulatedBodyInertia & operator()(const LinkIndex link);
        const iDynTree::ArticulatedBodyInertia & operator()(const LinkIndex link) const;

        ~LinkArticulatedBodyInertias();
    };

    /**
     * Class for storing a vector of twists, one for each link in a model.
     */
    class LinkVelArray
    {
    private:
        std::vector<iDynTree::Twist> m_linkTwist;

    public:
        LinkVelArray(unsigned int nrOfLinks = 0);
        LinkVelArray(const iDynTree::Model & model);

        void resize(unsigned int nrOfLinks);
        void resize(const iDynTree::Model & model);

        iDynTree::Twist & operator()(const LinkIndex link);
        const iDynTree::Twist & operator()(const LinkIndex link) const;

        ~LinkVelArray();
    };

    /**
     * Class for storing a vector of spatial accelerations,
     *  one for each link in a model.
     */
    class LinkAccArray
    {
    private:
        std::vector<iDynTree::SpatialAcc> m_linkAcc;

    public:
        LinkAccArray(unsigned int nrOfLinks = 0);
        LinkAccArray(const iDynTree::Model & model);

        void resize(unsigned int nrOfLinks);
        void resize(const iDynTree::Model & model);

        iDynTree::SpatialAcc & operator()(const LinkIndex link);
        const iDynTree::SpatialAcc & operator()(const LinkIndex link) const;

        unsigned int getNrOfLinks() const;

        ~LinkAccArray();
    };
}

#endif /* IDYNTREE_LINK_STATE_H */