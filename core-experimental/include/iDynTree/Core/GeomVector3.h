/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_GEOM_VECTOR_3_H
#define IDYNTREE_GEOM_VECTOR_3_H

#include <iDynTree/Core/VectorFixSize.h>
#include <typeinfo>

namespace iDynTree
{
    class Rotation;
    //class LinearMotionVector3;
    //class AngularMotionVector3;
    //class LinearForceVector3;
    //class AngularForceVector3;

    /**
     * Template class providing the raw coordinates and semantics for any geometric relation vector.
     *
     * \ingroup iDynTreeCore
     *
     * A geometrical vector can be used to describe a motion or a force vector.
     *
     * This is a basic vector, used to implement the adjoint transformations common
     * to motion and force vectors or to just provide an interface.
     *
     */
    template <class MotionForceT, template <class AnyMotionForceT> class MotionForceConversionsT>
    class GeomVector3: public Vector3
    {
    private:
        static GeomVector3<MotionForceT, MotionForceConversionsT> classSingleton;
        
        static MotionForceT& AliasMotionForceT() {
            return *static_cast<MotionForceT*>(classSingleton);
        }
        
    public:
        /**
         * constructors
         */
        GeomVector3();
        GeomVector3(const double* in_data, const unsigned int in_size);
        GeomVector3(const GeomVector3 & other);
        virtual ~GeomVector3();
        
        /**
         * Geometric operations
         */
        const MotionForceT & changeCoordFrame(const Rotation & newCoordFrame);
        static MotionForceT compose(const MotionForceT & op1, const MotionForceT & op2);
        static MotionForceT inverse(const MotionForceT & op);
        
        /**
         * dot product
         */
        double dot(const typename MotionForceConversionsT<Vector3>::DualSpace & other) const;
        
        /**
         * overloaded operators
         */
        MotionForceT operator+(const MotionForceT &other) const;
        MotionForceT operator-(const MotionForceT &other) const;
        MotionForceT operator-() const;
    };

    /**
     * Method definitions
     */
    
    // constructors
    template <class MotionForceT, template <class AnyMotionForceT> class MotionForceConversionsT>
    GeomVector3<MotionForceT, MotionForceConversionsT>::GeomVector3(): Vector3()
    {}
    
    template <class MotionForceT, template <class AnyMotionForceT> class MotionForceConversionsT>
    GeomVector3<MotionForceT, MotionForceConversionsT>::GeomVector3(const double* in_data, const unsigned int in_size): Vector3(in_data, in_size)
    {}
    
    template <class MotionForceT, template <class AnyMotionForceT> class MotionForceConversionsT>
    GeomVector3<MotionForceT, MotionForceConversionsT>::GeomVector3(const GeomVector3 & other): Vector3(other)
    {}
    
    template <class MotionForceT, template <class AnyMotionForceT> class MotionForceConversionsT>
    GeomVector3<MotionForceT, MotionForceConversionsT>::~GeomVector3()
    {}
    
    // Geometric operations
    template <class MotionForceT, template <class AnyMotionForceT> class MotionForceConversionsT>
    const MotionForceT & GeomVector3<MotionForceT, MotionForceConversionsT>::changeCoordFrame(const Rotation & newCoordFrame)
    {}
    
    template <class MotionForceT, template <class AnyMotionForceT> class MotionForceConversionsT>
    MotionForceT GeomVector3<MotionForceT, MotionForceConversionsT>::compose(const MotionForceT & op1, const MotionForceT & op2)
    {}
    
    template <class MotionForceT, template <class AnyMotionForceT> class MotionForceConversionsT>
    MotionForceT GeomVector3<MotionForceT, MotionForceConversionsT>::inverse(const MotionForceT & op)
    {}
    
    // dot product
    template <class MotionForceT, template <class AnyMotionForceT> class MotionForceConversionsT>
    double GeomVector3<MotionForceT, MotionForceConversionsT>::dot(const typename MotionForceConversionsT<Vector3>::DualSpace & other) const
    {}

    // overloaded operators
    template <class MotionForceT, template <class AnyMotionForceT> class MotionForceConversionsT>
    MotionForceT GeomVector3<MotionForceT, MotionForceConversionsT>::operator+(const MotionForceT &other) const
    {}
    
    template <class MotionForceT, template <class AnyMotionForceT> class MotionForceConversionsT>
    MotionForceT GeomVector3<MotionForceT, MotionForceConversionsT>::operator-(const MotionForceT &other) const
    {}
    
    template <class MotionForceT, template <class AnyMotionForceT> class MotionForceConversionsT>
    MotionForceT GeomVector3<MotionForceT, MotionForceConversionsT>::operator-() const
    {}
    
    /**
     * Possible instanciations
     */
    //typedef GeomVector3<LinearForceVector3> GeomVector3LF;
    //typedef GeomVector3<AngularForceVector3> GeomVector3AF;
}

#endif /* IDYNTREE_GEOM_VECTOR_3_H */