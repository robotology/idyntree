/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_ANGULAR_FORCE_VECTOR_3_H
#define IDYNTREE_ANGULAR_FORCE_VECTOR_3_H

#include <iDynTree/Core/ForceVector3.h>
#include <iDynTree/Core/PrivateMotionForceVertorAssociations.h>

namespace iDynTree
{
    /**
     * Class providing the raw coordinates and semantics for any torque vector
     *
     * \ingroup iDynTreeCore
     *
     * A motion vector can be used to describe a angular momentum or force,
     * and implement the adjoint transformations common to these geometric relations.
     *
     */
    class AngularForceVector3: public ForceVector3<AngularForceVector3, AngularForceAssociationsT>
    {
    public:
        /**
         * constructors
         */
        AngularForceVector3();
        AngularForceVector3(const double* in_data, const unsigned int in_size);
        AngularForceVector3(const AngularForceVector3 & other);
        virtual ~AngularForceVector3();
    };
}

#endif /* IDYNTREE_ANGULAR_FORCE_VECTOR_3_H */