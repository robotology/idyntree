/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_GEOM_VECTOR_3_H
#define IDYNTREE_GEOM_VECTOR_3_H

#include <iDynTree/Core/VectorFixSize.h>

namespace iDynTree
{
    class Rotation;
    class LinearMotionVector3;
    class AngularMotionVector3;
    class LinearForceVector3;
    class AngularForceVector3;

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
    template <typename T>
    struct GeomVector3: public Vector3
    {
        /**
         * constructors
         */
        GeomVector3();
        GeomVector3(const double* in_data, const unsigned int in_size);
        GeomVector3(const GeomVector3 & other);
        virtual ~GeomVector3();
        
        /* Geometric operations */
        const T & changeCoordFrame(const Rotation & newCoordFrame);
        static T compose(const T & op1, const T & op2);
        static T inverse(const T & op);
        
        /* dot product */
        //double dot(const typename T::dualSpace & other) const;
        
        /** overloaded operators **/
        T operator+(const T &other) const;
        T operator-(const T &other) const;
        T operator-() const;
        
        /** constructor helpers */
        static GeomVector3 Zero();
    };

    /**
     * Method definitions
     */
    template <typename T>
    class GeomVector3<T>::GeomVector3():
    {
    }
    
    template <typename T>
    class GeomVector3<T>::GeomVector3(const double* in_data, const unsigned int in_size): Vector3(in_data, in_size)
    {}
    
    template <typename T>
    class GeomVector3<T>::GeomVector3(const GeomVector3 & other): Vector3(other)
    {}
    
    /* Possible instanciations
     typedef GeomVector3<LinearForceVector3> GeomVector3LF;
     typedef GeomVector3<AngularForceVector3> GeomVector3AM;*/
    
}

#endif /* IDYNTREE_GEOM_VECTOR_3_H */