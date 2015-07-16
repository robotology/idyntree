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
     * Helper class only used along with class AngularMotionVector3 but defined outside it.
     * Check comments about CTRP technique.
     */
    template <class MotionForceT> class AngularMotionConvertionsT
    {
    public:
        /**
         * Helper type providing the associated class in the dual space (might be a vocabulary abuse)
         */
        typedef AngularForceVector3 DualSpace;
        
        /**
         * Helper type providing the class associated to the alternate type of movement in the same space
         */
        typedef LinearMotionVector3 InvertLinAng;
        
        /**
         * Helper class providing the result class of the cross product for the operator (w\times).
         */
        template <class MotionOrForceT=MotionForceT>
        struct Derivative
        {
            typedef class MotionOrForceT Type;
        };
        
    };

    /**
     * Class providing the raw coordinates and semantics for any angular motion vector
     *
     * \ingroup iDynTreeCore
     *
     * An angular motion vector can be used to describe any angular velocity or acceleration,
     * and implement the adjoint transformations common to these geometric relations.
     *
     */
    class AngularMotionVector3: public MotionVector3<AngularMotionVector3, AngularMotionConvertionsT>
    {
    public:
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