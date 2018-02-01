/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_TRAVERSAL_H
#define IDYNTREE_TRAVERSAL_H

#include <iDynTree/Model/Indices.h>

#include <vector>

namespace iDynTree
{
    class IJoint;
    class Link;
    class Model;

    /**
     * Class that represents a traversal of a set of links of a Model.
     * The traversal is represented by an ordered vector of links.
     * For a given model, the traversal is always built in such a way
     * that there exist a spanning tree that has as a root the first link
     * (with traversal index 0) of the traversal, and such that every link comes
     * after its spanning tree parent.
     *
     * For every link in the traversal are provided:
     *  * a pointer to the link itself
     *  * a pointer to the parent link in the spanning tree
     *  * a pointer to the joint connecting the link to the parent
     *
     * For the first link of the traversal (i.e. base of the traversal)
     * there is not spanning tree parent, so the point to the parent link/joint are NULL.
     *
     *
     * \ingroup iDynTreeModel
     */
    class Traversal
    {
    private:
        std::vector<const Link   *> links;
        std::vector<const Link   *> parents;
        std::vector<const IJoint *> toParentJoints;
        std::vector<int>            linkIndexToTraversalIndex;

        /**
         * Copy constructor is forbidden
         */
        Traversal(const Traversal & other);

        /**
         * Copy operator is forbidden
         */
        Traversal& operator=(const Traversal &other);

    public:
        Traversal();

        ~Traversal();

        /**
         * Get the number of visited links.
         *
         * @return the number of links in the Traversal
         */
        unsigned int getNrOfVisitedLinks() const;

        /**
         * Get the traversalIndex-th link of the traversal.
         *
         * @return a pointer to the traversalIndex-th link of the traversal.
         */
        const Link   * getLink(const TraversalIndex traversalIndex) const;

        /**
         * Get the base link of the traversal.
         *
         * @note this is equivalent to getLink(0), as the base link
         *       is by definition the first link of the traversal.
         * @return a pointer to the base link of the traversal.
         */
        const Link   * getBaseLink() const;

        /**
         * Get the parent link of the traversalIndex-th link of the traversal.
         *
         * @return a pointer to the parent link of the traversalIndex-th link of the traversal.
         */
        const Link   * getParentLink(const TraversalIndex traversalIndex) const;

        /**
         * Get the joint connecting the traversalIndex-th link of the traversal
         * to its parent.
         *
         * @return a pointer to the joint connecting the  link traversalIndex-th link of the traversal.
         */
        const IJoint * getParentJoint(const TraversalIndex traversalIndex) const;

        /**
         * Get the parent link of the link with index linkIndex of the traversal.
         *
         * @return a pointer to the parent link of the traversalIndex-th link of the traversal.
         */
        const Link   * getParentLinkFromLinkIndex(const LinkIndex linkIndex) const;

        /**
         * Get the joint connecting the link with index linkIndex
         * to its parent.
         *
         * @return a pointer to the joint connecting the  link traversalIndex-th link of the traversal.
         */
        const IJoint * getParentJointFromLinkIndex(const LinkIndex linkIndex) const;

        /**
         * Get the traversal index of the specified link
         *
         * @return the traversalIndex of the specified link, or TRAVERSAL_INDEX_INVALID if the link does not belong to the traversal.
         */
        TraversalIndex getTraversalIndexFromLinkIndex(const LinkIndex linkIndex) const;



        /**
         * Reset the Traversal.
         *
         * After a call to reset, the Traversal will contain
         * no visited links, so it needs to be approprately populated
         * with the addTraversalBase and addTraversalElement methods.
         *
         * @param[in] nrOfLinksInModel total number of links in the model,
         *                             not the number of visited links in the Traversal.
         * @return true if all went well, false otherwise
         */
        bool reset(const unsigned int nrOfLinksInModel);

        /**
         * Reset the Traversal.
         *
         * After a call to reset, all the pointers in the Traversal are set
         * to 0, and the Traversal should be approprialy populated with the
         * setters before use.
         *
         * @param[in] model Model on which this traversal will be used,
         *                  used to initialize the internal data structure
         *                  using the getNrOfLinks() method.
         *
         * @return true if all went well, false otherwise
         */
        bool reset(const Model & model);

        /**
         * Add a base to traversal.
         *
         * Equivalent to addTraversalElement(link,NULL,NULL), but
         * will print an error and return false if it is called with a
         * traversal that already has a base, i.e. getNrOfVisitedLinks() > 0
         *
         * @param[in] link a pointer to the base link
         * @return true if all went well, false otherwise
         */
        bool addTraversalBase(const Link * link);

        /**
         * Add an element to the traversal.
         *
         * After a call to this method, the getNrOfVisitedLinks()
         * will be increased of 1 unit.
         *
         * @param[in] link a pointer to the link to add to the traversal
         * @param[in] jointToParent a pointer to the joint that connects
         *                          the added link to its parent in this traversal
         * @param[in] parentLink a pointer to the parent on this link in this traversal.
         *                       It should be a link already contained in this traversal.
         * @return true if all went well, false otherwise.
         */
        bool addTraversalElement(const Link   * link,
                                 const IJoint * jointToParent,
                                 const Link   * parentLink);

        /**
         * \brief Check if a link is the parent of another link for this traversal.
         *
         *
         * @param[in] parentCandidate the link candidate to be the parent of childCandidate.
         * @param[in] childCandidate  the link candidate to be a child of parentCandidate.
         * @return true if parentCandidate is actually the parent of childCandidate, false otherwise.
         */
        bool isParentOf(const LinkIndex parentCandidate, const LinkIndex childCandidate) const;

        /**
         * \brief Get the child link (according to the traversal) of a Joint.
         *
         * @param[in] m_model the considered model.
         * @param[in] jntIdx the index of the joint of which we want to get the child link.
         * @return the index of the child link if all went well, LINK_INVALID_INDEX otherwise .
         */
        LinkIndex getChildLinkIndexFromJointIndex(const Model & model, const JointIndex jntIdx) const;

        /**
         * \brief Get the parent link (according to the traversal) of a Joint.
         *
         * @param[in] m_model the considered model.
         * @param[in] jntIdx the index of the joint of which we want to get the parent link.
         * @return the index of the parent link if all went well, LINK_INVALID_INDEX otherwise .
         */
        LinkIndex getParentLinkIndexFromJointIndex(const Model & model, const JointIndex jntIdx) const;

        /**
         * Return a human-readable representation of the traversal.
         */
        std::string toString(const Model & model) const;

    };


}

#endif /* IDYNTREE_TRAVERSAL_H */
