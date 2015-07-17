/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_PRIVATE_MOTION_FORCE_VECTOR_ASSOCIATIONS_H
#define IDYNTREE_PRIVATE_MOTION_FORCE_VECTOR_ASSOCIATIONS_H


namespace iDynTree
{
    class LinearMotionVector3;
    class AngularMotionVector3;
    class LinearForceVector3;
    class AngularForceVector3;
    
    
    /**
     * Helper class only used along with class AngularForceVector3 but defined outside this
     * same class because we are using the CRTP technique (Curiously Recurring Template Pattern).
     * CRTP here results in the Base class ForceVector3 being instanciated before AngularForceVector3,
     * with this same class as template parameter. The goal is too define methods whose bodies
     * are specific to class ForceVector3 (so common to AngularForceVector3 and LinearForceVector3),
     * but the returned type is specific to AngularForceVector3.
     */
    class AngularForceAssociationsT
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
     * Helper class only used along with class AngularMotionVector3 but defined outside it.
     * Check comments about CTRP technique.
     */
    class AngularMotionAssociationsT
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
        template <class MotionForceT>
        struct Derivative
        {
            typedef MotionForceT Type;
        };
        
    };
    
    /**
     * Helper class only used along with class LinearForceVector3 but defined outside it.
     * Check comments about CTRP technique.
     */
    class LinearForceAssociationsT
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
     * Helper class only used along with class LinearMotionVector3 but defined outside it.
     * Check comments about CTRP technique.
     */
    class LinearMotionAssociationsT
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
        template <class MotionForceT>
        struct Derivative
        {
            typedef typename MotionForceT::InvertLinAng Type;
        };
        
    };
}

#endif /* IDYNTREE_PRIVATE_MOTION_FORCE_VECTOR_ASSOCIATIONS_H */