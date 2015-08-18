/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_LINK_H
#define IDYNTREE_LINK_H

#include <iDynTree/Core/SpatialInertia.h>

#include <iDynTree/Model/Indeces.h>


namespace iDynTree
{

    /**
     * Class that represents a link.
     *
     *
     * \ingroup iDynTreeCore
     */
    class Link
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
    };


}

#endif /* IDYNTREE_LINK_H */