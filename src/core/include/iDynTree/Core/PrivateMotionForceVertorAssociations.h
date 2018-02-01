/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_PRIVATE_MOTION_FORCE_VECTOR_ASSOCIATIONS_H
#define IDYNTREE_PRIVATE_MOTION_FORCE_VECTOR_ASSOCIATIONS_H


namespace iDynTree
{
    class LinearMotionVector3;
    class AngularMotionVector3;
    class LinearForceVector3;
    class AngularForceVector3;
    class AngularForceVector3Semantics;
    class AngularMotionVector3Semantics;
    class LinearForceVector3Semantics;
    class LinearMotionVector3Semantics;


    /**
     * This is a custom traits class. A traits class class_traits<class T> is a template class
     * defining some attributes (i.e. properties, traits) of the class parameter T.
     * We are using CRTP (Curiously Recurring Template Pattern) from which arises some compile issues.
     * For instance, AngularForceVector3 is derived in CRTP way from ForceVector3<AngularForceVector3>.
     * So, at compile time, the base class is being instanciated before the derived class, but having
     * to access the derived class which is not yet fuly defined. For solving this issue we define 
     * the below traits-like class gathering all these early accessed fields.
     */
    template <class MotionForceT>
    class MotionForce_traits {};
    
    /**
     * Traits class instance only used along with class AngularForceVector3
     */
    template <>
    class MotionForce_traits<AngularForceVector3>
    {
    public:
        /**
         * Helper types providing the class and the associated dual space class (might be a vocabulary abuse)
         */
        typedef AngularForceVector3  Type;
        typedef AngularMotionVector3 DualSpace;
        
        /**
         * Helper type providing the associated semantics class
         */
        typedef AngularForceVector3Semantics SemanticsType;

        /**
         * Helper type providing the class associated to the alternate type of movement in the same space
         */
        typedef LinearForceVector3  DerivByLinearMotion;
        typedef AngularForceVector3 DerivByAngularMotion;
    };
    
    /**
     * Traits class instance only used along with class AngularMotionVector3
     */
    template <>
    class MotionForce_traits<AngularMotionVector3>
    {
    public:
        /**
         * Helper types providing the class and the associated dual space class (might be a vocabulary abuse)
         */
        typedef AngularMotionVector3 Type;
        typedef AngularForceVector3  DualSpace;
        
        /**
         * Helper type providing the associated semantics class
         */
        typedef AngularMotionVector3Semantics SemanticsType;
        
        /**
         * Helper type providing the class associated to the alternate type of movement in the same space
         */
        typedef LinearMotionVector3  DerivByLinearMotion;
        typedef AngularMotionVector3 DerivByAngularMotion;
    };
    
    /**
     * Traits class instance only used along with class LinearForceVector3
     */
    template <>
    class MotionForce_traits<LinearForceVector3>
    {
    public:
        /**
         * Helper types providing the class and the associated dual space class (might be a vocabulary abuse)
         */
        typedef LinearForceVector3  Type;
        typedef LinearMotionVector3 DualSpace;
        
        /**
         * Helper type providing the associated semantics class
         */
        typedef LinearForceVector3Semantics SemanticsType;
        
        /**
         * Helper type providing the class associated to the alternate type of movement in the same space
         */
        typedef AngularForceVector3 DerivByLinearMotion;
        typedef LinearForceVector3 DerivByAngularMotion;
    };
    
    /**
     * Traits class instance only used along with class LinearMotionVector3
     */
    template <>
    class MotionForce_traits<LinearMotionVector3>
    {
    public:
        /**
         * Helper types providing the class and the associated dual space class (might be a vocabulary abuse)
         */
        typedef LinearMotionVector3 Type;
        typedef LinearForceVector3  DualSpace;
        
        /**
         * Helper type providing the associated semantics class
         */
        typedef LinearMotionVector3Semantics SemanticsType;
        
        /**
         * Helper type providing the class associated to the alternate type of movement in the same space
         */
        typedef AngularMotionVector3 DerivByLinearMotion;
        typedef LinearMotionVector3 DerivByAngularMotion;
    };

    
    
    /**
     * Helper class providing the result class of the cross product for the operator (\f$w\times\f$ or \f$v\times\f$).
     * (AngularMotionVector3 x MotionForce or LinearMotionVector3 x MotionForce)
     */
    template <class MotionT, class MotionForce2deriveT>
    struct MotionDerivativeOf {};

    template <class MotionForce2deriveT>
    struct MotionDerivativeOf<AngularMotionVector3, MotionForce2deriveT>
    {
        typedef typename MotionForce_traits<MotionForce2deriveT>::DerivByAngularMotion Type;
    };

    template <class MotionForce2deriveT>
    struct MotionDerivativeOf<LinearMotionVector3, MotionForce2deriveT>
    {
        typedef typename MotionForce_traits<MotionForce2deriveT>::DerivByLinearMotion Type;
    };


    /**
     * convertion from semantics classes to Motion/Force traits specializations
     */
    template <class MotionForceSemanticsT>
    struct ConvertSem2motionForceTraits {};

    template <>
    struct ConvertSem2motionForceTraits<AngularForceVector3Semantics>
    {
        typedef MotionForce_traits<AngularForceVector3> Traits;
    };

    template <>
    struct ConvertSem2motionForceTraits<AngularMotionVector3Semantics>
    {
        typedef MotionForce_traits<AngularMotionVector3> Traits;
    };
    
    template <>
    struct ConvertSem2motionForceTraits<LinearForceVector3Semantics>
    {
        typedef MotionForce_traits<LinearForceVector3> Traits;
    };
    
    template <>
    struct ConvertSem2motionForceTraits<LinearMotionVector3Semantics>
    {
        typedef MotionForce_traits<LinearMotionVector3> Traits;
    };
    
    template <class MotionForceSemanticsT>
    struct DualMotionForceSemanticsT
    {
        typedef typename MotionForce_traits<typename ConvertSem2motionForceTraits<MotionForceSemanticsT>::Traits::DualSpace>::SemanticsType Type;
    };
    
    /**
     * Traits class for SpatialMotionVector and SpatialForceVector classes
     */
    template <class SpatialMotionForceVectorT>
    class SpatialMotionForceVectorT_traits {};

    template <>
    class SpatialMotionForceVectorT_traits<SpatialMotionVector>
    {
    public:
        typedef LinearMotionVector3 LinearVector3Type;
        typedef AngularMotionVector3 AngularVector3Type;
    };

    template <>
    class SpatialMotionForceVectorT_traits<SpatialForceVector>
    {
    public:
        typedef LinearForceVector3 LinearVector3Type;
        typedef AngularForceVector3 AngularVector3Type;
    };
}

#endif /* IDYNTREE_PRIVATE_MOTION_FORCE_VECTOR_ASSOCIATIONS_H */
