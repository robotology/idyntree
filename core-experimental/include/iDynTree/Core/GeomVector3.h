/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_GEOM_VECTOR_3_H
#define IDYNTREE_GEOM_VECTOR_3_H

#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/PrivateMotionForceVertorAssociations.h>

namespace iDynTree
{
    class Rotation;

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
    template <class MotionForceT, class MotionForceAssociationsT>
    class GeomVector3: public Vector3
    {
    private:
        static GeomVector3<MotionForceT, MotionForceAssociationsT> classSingleton;
        
        static MotionForceT& AliasMotionForceT() {
            return static_cast<MotionForceT&>(classSingleton);
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
        double dot(const typename MotionForceAssociationsT::DualSpace & other) const;
        
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
    template <class MotionForceT, class MotionForceAssociationsT>
    GeomVector3<MotionForceT, MotionForceAssociationsT>::GeomVector3(): Vector3()
    {}
    
    template <class MotionForceT, class MotionForceAssociationsT>
    GeomVector3<MotionForceT, MotionForceAssociationsT>::GeomVector3(const double* in_data, const unsigned int in_size): Vector3(in_data, in_size)
    {}
    
    template <class MotionForceT, class MotionForceAssociationsT>
    GeomVector3<MotionForceT, MotionForceAssociationsT>::GeomVector3(const GeomVector3 & other): Vector3(other)
    {}
    
    template <class MotionForceT, class MotionForceAssociationsT>
    GeomVector3<MotionForceT, MotionForceAssociationsT>::~GeomVector3()
    {}
    
    // Geometric operations
    template <class MotionForceT, class MotionForceAssociationsT>
    const MotionForceT & GeomVector3<MotionForceT, MotionForceAssociationsT>::changeCoordFrame(const Rotation & newCoordFrame)
    {}
    
    template <class MotionForceT, class MotionForceAssociationsT>
    MotionForceT GeomVector3<MotionForceT, MotionForceAssociationsT>::compose(const MotionForceT & op1, const MotionForceT & op2)
    {}
    
    template <class MotionForceT, class MotionForceAssociationsT>
    MotionForceT GeomVector3<MotionForceT, MotionForceAssociationsT>::inverse(const MotionForceT & op)
    {}
    
    // dot product
    template <class MotionForceT, class MotionForceAssociationsT>
    double GeomVector3<MotionForceT, MotionForceAssociationsT>::dot(const typename MotionForceAssociationsT::DualSpace & other) const
    {}

    // overloaded operators
    template <class MotionForceT, class MotionForceAssociationsT>
    MotionForceT GeomVector3<MotionForceT, MotionForceAssociationsT>::operator+(const MotionForceT &other) const
    {}
    
    template <class MotionForceT, class MotionForceAssociationsT>
    MotionForceT GeomVector3<MotionForceT, MotionForceAssociationsT>::operator-(const MotionForceT &other) const
    {}
    
    template <class MotionForceT, class MotionForceAssociationsT>
    MotionForceT GeomVector3<MotionForceT, MotionForceAssociationsT>::operator-() const
    {}
}

#endif /* IDYNTREE_GEOM_VECTOR_3_H */