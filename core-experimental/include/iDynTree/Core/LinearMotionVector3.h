/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_LINEAR_MOTION_VECTOR_3_H
#define IDYNTREE_LINEAR_MOTION_VECTOR_3_H

#include <iDynTree/Core/MotionVector3.h>

namespace iDynTree
{
    class LinearForceVector3;
    class AngularMotionVector3;
    
    /**
     * Helper class only used along with class LinearMotionVector3 but defined outside it.
     * Check comments about CTRP technique.
     */
    template <class MotionForceT> class LinearMotionConvertionsT
    {
    public:
        /**
         * Helper type providing the associated class in the dual space (might be a vocabulary abuse)
         */
        typedef LinearForceVector3 DualSpace;
        
        /**
         * Helper type providing the class associated to the alternate type of movement in the same space
         */
        typedef AngularMotionVector3 InvertLinAng;

        /**
         * Helper class providing the result class of the cross product for the operator (v\times).
         */
        template <class MotionOrForceT=MotionForceT>
        struct Derivative
        {
            typedef class MotionOrForceT::invertLinAng Type;
        };
        
    };
    
    /**
     * Class providing the raw coordinates and semantics for any linear motion vector
     *
     * \ingroup iDynTreeCore
     *
     * A linear motion vector can be used to describe any linear velocity or acceleration,
     * and implement the adjoint transformations common to these geometric relations.
     *
     */
    class LinearMotionVector3: public MotionVector3<LinearMotionVector3, LinearMotionConvertionsT>
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