/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_LINEAR_MOTION_VECTOR_3_H
#define IDYNTREE_LINEAR_MOTION_VECTOR_3_H

#include <iDynTree/Core/MotionVector3.h>
#include <iDynTree/Core/PrivateMotionForceVertorAssociations.h>

namespace iDynTree
{
    /**
     * Class providing the raw coordinates and semantics for any linear motion vector
     *
     * \ingroup iDynTreeCore
     *
     * A linear motion vector can be used to describe any linear velocity or acceleration,
     * and implement the adjoint transformations common to these geometric relations.
     *
     */
    class LinearMotionVector3: public MotionVector3<LinearMotionVector3, LinearMotionAssociationsT>
    {
    public:
        /**
         * constructors
         */
        LinearMotionVector3();
        LinearMotionVector3(const double* in_data, const unsigned int in_size);
        LinearMotionVector3(const LinearMotionVector3 & other);
        virtual ~LinearMotionVector3();
    };
}

#endif /* IDYNTREE_LINEAR_MOTION_VECTOR_3_H */