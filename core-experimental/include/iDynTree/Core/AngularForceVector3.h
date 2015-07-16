/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_ANGULAR_FORCE_VECTOR_3_H
#define IDYNTREE_ANGULAR_FORCE_VECTOR_3_H

#include <iDynTree/Core/ForceVector3.h>

namespace iDynTree
{
    class AngularMotionVector3;
    class LinearForceVector3;
    
    /**
     * Helper class only used along with class AngularForceVector3 but defined outside this
     * same class because we are using the CRTP technique (Curiously Recurring Template Pattern).
     * CRTP here results in the Base class ForceVector3 being instanciated before AngularForceVector3,
     * with this same class as template parameter. The goal is too define methods whose bodies
     * are specific to class ForceVector3 (so common to AngularForceVector3 and LinearForceVector3), 
     * but the returned type is specific to AngularForceVector3.
     */
    template <class MotionForceT> class AngularForceConvertionsT
    {
    public:
        /**
         * Helper type providing the associated class in the dual space (might be a vocabulary abuse)
         */
        typedef AngularMotionVector3 DualSpace;
        
        /**
         * Helper type providing the class associated to the alternate type of movement in the same space
         */
        typedef LinearForceVector3 InvertLinAng;
    };
    
    /**
     * Class providing the raw coordinates and semantics for any torque vector
     *
     * \ingroup iDynTreeCore
     *
     * A motion vector can be used to describe a angular momentum or force,
     * and implement the adjoint transformations common to these geometric relations.
     *
     */
    class AngularForceVector3: public ForceVector3<AngularForceVector3, AngularForceConvertionsT>
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