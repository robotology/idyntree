// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef IDYNTREE_LINK_H
#define IDYNTREE_LINK_H

#include <iDynTree/SpatialInertia.h>

#include <iDynTree/Indices.h>


namespace iDynTree
{

    /**
     * Class that represents a link.
     *
     *
     * \ingroup iDynTreeModel
     */
    class Link
    {
    private:
        LinkIndex m_index;
        SpatialInertia m_inertia;

    public:
        /**
         * Costructor
         */
        Link();

        /**
         * Low-level accessor to the link inertia.
         */
        SpatialInertia & inertia();


        /**
         * Low-level const accessor to the link inertia.
         */
        const SpatialInertia & inertia() const;

        /**
         * Set the spatial inertia of the link,
         * expressed in the link reference frame
         * (i.e. with respect to the link reference
         *  frame origin and with the orientation
         *  of the link reference frame).
         */
        void setInertia(SpatialInertia & _inertia);

        /**
         * Get the spatial inertia of the link,
         * expressed in the link reference frame
         * (i.e. with respect to the link reference
         *  frame origin and with the orientation
         *  of the link reference frame).
         *
         * @return a reference to the inertia of the link.
         */
        const SpatialInertia & getInertia() const;

        /**
         * Set the index of the link.
         */
        void setIndex(LinkIndex & _index);

        /**
         * Get the index of the link.
         */
        LinkIndex getIndex() const;
    };

    typedef Link * LinkPtr;
    typedef const Link * LinkConstPtr;
}

#endif /* IDYNTREE_LINK_H */
