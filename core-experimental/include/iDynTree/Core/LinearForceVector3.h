/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_LINEAR_FORCE_VECTOR_3_H
#define IDYNTREE_LINEAR_FORCE_VECTOR_3_H

#include <iDynTree/Core/ForceVector3.h>

namespace iDynTree
{
    class LinearMotionVector3;
    class AngularForceVector3;
    
    /**
     * Helper class only used along with class LinearForceVector3 but defined outside it.
     * Check comments about CTRP technique.
     */
    template <class MotionForceT> class LinearForceConvertionsT
    {
    public:
        /**
         * Helper type providing the associated class in the dual space (might be a vocabulary abuse)
         */
        typedef LinearMotionVector3 DualSpace;
        
        /**
         * Helper type providing the class associated to the alternate type of movement in the same space
         */
        typedef AngularForceVector3 InvertLinAng;
    };
    
    /**
     * Class providing the raw coordinates and semantics for any linear force vector
     *
     * \ingroup iDynTreeCore
     *
     * A force vector can be used to describe any linear momentum or force,
     * and implement the adjoint transformations common to these geometric relations.
     *
     */
    class LinearForceVector3: public ForceVector3<LinearForceVector3, LinearForceConvertionsT>
    {
    public:
        /**
         * constructors
         */
        LinearForceVector3();
        LinearForceVector3(const double* in_data, const unsigned int in_size);
        LinearForceVector3(const LinearForceVector3 & other);
        virtual ~LinearForceVector3();
    };
}

#endif /* IDYNTREE_LINEAR_FORCE_VECTOR_3_H */