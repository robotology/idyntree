/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_FORCE_VECTOR_3_H
#define IDYNTREE_FORCE_VECTOR_3_H

#include <iDynTree/Core/GeomVector3.h>

namespace iDynTree
{
    /**
     * Class providing the raw coordinates and semantics for any force vector
     *
     * \ingroup iDynTreeCore
     *
     * A force vector can be used to describe a linear or angular momentum or force.
     *
     * This is a basic vector, used to implement the adjoint transformations common
     * to every force vectors.
     *
     */
    template <class ForceT>
    class ForceVector3: public GeomVector3<ForceT>
    {
    public:
        /**
         * constructors
         */
        ForceVector3();
        ForceVector3(const double* in_data, const unsigned int in_size);
        ForceVector3(const ForceVector3 & other);
        virtual ~ForceVector3();
    };
}

#endif /* IDYNTREE_FORCE_VECTOR_3_H */