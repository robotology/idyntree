/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_TRAVERSAL_H
#define IDYNTREE_TRAVERSAL_H

#include <vector>

namespace iDynTree
{
    class IJoint;
    class Link;

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
        std::vector<Link   *> links;
        std::vector<Link   *> parents;
        std::vector<IJoint *> toParentJoints;

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
        Link   * getLink(unsigned int traversalIndex) const;

        /**
         * Get the parent link of the traversalIndex-th link of the traversal.
         *
         * @return a pointer to the parent link of the traversalIndex-th link of the traversal.
         */
        Link   * getParentLink(unsigned int traversalIndex) const;

        /**
         * Get the joint connecting the traversalIndex-th link of the traversal
         * to its parent.
         *
         * @return a pointer to the joint connecting the  link traversalIndex-th link of the traversal.
         */
        IJoint * getParentJoint(unsigned int traversalIndex) const;

        /**
         * Reset the Traversal to contain nrOfVisitedLinks visited links.
         *
         * After a call to reset, all the pointers in the Traversal are set
         * to 0, and the Traversal should be approprialy populated with the
         * setters before use.
         *
         * @return true if all went well, false otherwise
         */
        bool reset(unsigned int nrOfVisitedLinks);

        /**
         *
         * @return true if all went well, false otherwise
         */
        bool setTraversalElement(unsigned int traversalIndex, Link * link, IJoint * jointToParent, Link * parentLink);

    };


}

#endif /* IDYNTREE_TRAVERSAL_H */