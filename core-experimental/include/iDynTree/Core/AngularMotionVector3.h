/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_ANGULAR_MOTION_VECTOR_3_H
#define IDYNTREE_ANGULAR_MOTION_VECTOR_3_H

#include <iDynTree/Core/MotionVector3.h>

namespace iDynTree
{
    class AngularForceVector3;
    class LinearMotionVector3;
    class MotionVector3;
    /**
     * Class providing the raw coordinates and semantics for any angular motion vector
     *
     * \ingroup iDynTreeCore
     *
     * An angular motion vector can be used to describe any angular velocity or acceleration,
     * and implement the adjoint transformations common to these geometric relations.
     *
     */
    class AngularMotionVector3: public MotionVector3<AngularMotionVector3>
    {
    public:
        /**
         * Helper type providing the associated class in the dual space (might be a vocabulary abuse)
         */
        typedef AngularForceVector3 dualSpace;
        
        /**
         * Helper type providing the class associated to the alternate type of movement in the same space
         */
        typedef LinearMotionVector3 invertLinAng;
        
        /**
         * Helper class providing the result class of the cross product
         */
        template <class MotionOrForceT>
        struct derivative
        {
            typedef MotionOrForceT Type;
        };
        
        /**
         * constructors
         */
        AngularMotionVector3();
        AngularMotionVector3(const double* in_data, const unsigned int in_size);
        AngularMotionVector3(const AngularMotionVector3 & other);
        virtual ~AngularMotionVector3();
    };
}

#endif /* IDYNTREE_ANGULAR_MOTION_VECTOR_3_H */