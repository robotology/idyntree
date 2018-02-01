/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_LINK_H
#define IDYNTREE_LINK_H

#include <iDynTree/Core/SpatialInertia.h>

#include <iDynTree/Model/Indices.h>


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
