// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_LINK_STATE_H
#define IDYNTREE_LINK_STATE_H

#include <iDynTree/Transform.h>
#include <iDynTree/Twist.h>
#include <iDynTree/SpatialAcc.h>
#include <iDynTree/Wrench.h>
#include <iDynTree/ArticulatedBodyInertia.h>

#include <iDynTree/Indices.h>

#include <vector>

namespace iDynTree
{
    class Model;

    class LinkPositions
    {
    private:
        std::vector<iDynTree::Transform> m_linkPos;

    public:
        LinkPositions(std::size_t nrOfLinks = 0);
        LinkPositions(const iDynTree::Model & model);

        void resize(std::size_t nrOfLinks);
        void resize(const iDynTree::Model & model);

        bool isConsistent(const Model& model) const;

        size_t getNrOfLinks() const;

        iDynTree::Transform & operator()(const LinkIndex link);
        const iDynTree::Transform & operator()(const LinkIndex link) const;

        std::string toString(const Model & model) const;

        ~LinkPositions();
    };

    /**
     * Vector of wrenches connected in some way to the link of a model.
     *
     * It is used to model both the total external wrench
     * acting on a  link (LinkExternalWrenches), or the internal wrenches
     * that a link excerts on his parent (given a Traversal)
     * computed as a by product by the dynamic loop of the RNEA ( RNEADynamicPhase ).
     *
     * In both cases the Wrench corresponding to the link with LinkIndex i
     * is always expressed with the orientation of the link frame and with
     * respect to the link frame origin.
     */
    class LinkWrenches
    {
    private:
        std::vector<iDynTree::Wrench> m_linkWrenches;

    public:
        /**
         * Create a LinkWrenches vector, with the size given
         * by nrOfLinks .
         *
         * @param[in] nrOfLinks the size of the vector.
         */
        LinkWrenches(std::size_t nrOfLinks = 0);
        LinkWrenches(const iDynTree::Model & model);

        /**
         * Resize the vector to have size nrOfLinks.
         *
         * @param[in] nrOfLinks new size for the vector
         */
        void resize(std::size_t nrOfLinks);
        void resize(const iDynTree::Model & model);

        bool isConsistent(const Model& model) const;
        size_t getNrOfLinks() const;

        iDynTree::Wrench & operator()(const LinkIndex link);
        const iDynTree::Wrench & operator()(const LinkIndex link) const;

        std::string toString(const Model & model) const;

        /**
         * Set all the elements to zero.
         */
        void zero();

        ~LinkWrenches();
    };

    /**
     * Vector of the sum of all the  external wrenches excerted on each link.
     *
     * The wrench returned by operator(i) is the sum of all external wrenches
     * (thus excluding the wrench applied on the link by other links in the model)
     * that the environment applies on the link $i$, expressed (
     * both orientation and point) with respect to the reference frame of link i.
     */
    typedef LinkWrenches LinkNetExternalWrenches;

    /**
     * Vector of the wrenches acting that a link excert on his parent,
     * given a Traversal.
     *
     * Given a Traversal with base link b, the wrench returned by operator(i) is the wrench
     * the parent of link i excerts on link i, expressed (both orientation
     * and point) with respect to the reference frame of link i.
     */
    typedef LinkWrenches LinkInternalWrenches;


    /**
     * Vector of the sum of all the wrenches (both internal and external, excluding gravity) acting on
     * link i, expressed (both orientation and point) with respect to the reference frame of link i.
     *
     * This is tipically computed as I*a+v*(I*v) , where a is the proper acceleration.
     */
    typedef LinkWrenches LinkNetTotalWrenchesWithoutGravity;

    /**
     * Class for storing a vector of SpatialInertia objects , one for each link in a model.
     */
    class LinkInertias
    {
    private:
        std::vector<iDynTree::SpatialInertia> m_linkInertials;

    public:
        LinkInertias(std::size_t nrOfLinks = 0);
        LinkInertias(const iDynTree::Model & model);

        void resize(std::size_t nrOfLinks);
        void resize(const iDynTree::Model & model);

        bool isConsistent(const Model& model) const;

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
        LinkArticulatedBodyInertias(std::size_t nrOfLinks = 0);
        LinkArticulatedBodyInertias(const iDynTree::Model & model);

        void resize(std::size_t nrOfLinks);
        void resize(const iDynTree::Model & model);

        bool isConsistent(const Model& model) const;

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
        LinkVelArray(std::size_t nrOfLinks = 0);
        LinkVelArray(const iDynTree::Model & model);

        void resize(std::size_t nrOfLinks);
        void resize(const iDynTree::Model & model);

        bool isConsistent(const Model& model) const;

        size_t getNrOfLinks() const;

        iDynTree::Twist & operator()(const LinkIndex link);
        const iDynTree::Twist & operator()(const LinkIndex link) const;

        std::string toString(const Model & model) const;

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
        LinkAccArray(std::size_t nrOfLinks = 0);
        LinkAccArray(const iDynTree::Model & model);

        void resize(std::size_t nrOfLinks);
        void resize(const iDynTree::Model & model);

        bool isConsistent(const Model& model) const;

        iDynTree::SpatialAcc & operator()(const LinkIndex link);
        const iDynTree::SpatialAcc & operator()(const LinkIndex link) const;

        std::size_t getNrOfLinks() const;

        std::string toString(const Model & model) const;

        ~LinkAccArray();
    };

    /**
     * Typedef used when the vector is meant to be a vector of link proper accelerations. 
     */
    typedef LinkAccArray LinkProperAccArray;
}

#endif /* IDYNTREE_LINK_STATE_H */
