/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_FRAME_H
#define IDYNTREE_FRAME_H

#include <iDynTree/Core/Transform.h>

#include <iDynTree/Model/Indeces.h>

namespace iDynTree
{

    /**
     * Class that represents an additional frame attached to a link.
     *
     * \ingroup iDynTreeCore
     */
    class AdditionalFrame
    {
    private:
        SpatialInertia m_inertia;

    public:
        /**
         * Costructor
         */
        Link();

        /**
         * Destructor
         *
         */
        virtual ~Link();

        /**
         * Set the spatial inertia of the link,
         * expressed in the link reference frame
         * (i.e. with respect to the link reference
         *  frame origin and with the orientation
         *  of the link reference frame).
         *
         *
         */
        virtual void setInertia(SpatialInertia & _inertia);

        /**
         * Get the spatial inertia of the link,
         * expressed in the link reference frame
         * (i.e. with respect to the link reference
         *  frame origin and with the orientation
         *  of the link reference frame).
         *
         * @return a reference to the inertia of the link.
         */
        virtual const SpatialInertia & getInertia() const;

        /**
         * Set the index of the link.
         */
        virtual void setIndex(LinkIndex & _index);

        /**
         * Get the index of the link.
         */
        virtual LinkIndex getIndex() const;
    };

    typedef Link * LinkPtr;
    typedef const Link * LinkConstPtr;
}

#endif /* IDYNTREE_LINK_H */