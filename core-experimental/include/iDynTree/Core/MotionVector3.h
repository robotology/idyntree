/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_MOTION_VECTOR_3_H
#define IDYNTREE_MOTION_VECTOR_3_H

#include <iDynTree/Core/GeomVector3.h>
#include <iDynTree/Core/PrivateMotionForceVertorAssociations.h>

namespace iDynTree
{
    /**
     * Class providing the raw coordinates and semantics for any motion vector
     *
     * \ingroup iDynTreeCore
     *
     * A motion vector can be used to describe a linear or angular velocity or acceleration.
     *
     * This is a basic vector, used to implement the adjoint transformations common
     * to every motion vectors.
     *
     */
    template <class MotionT, class MotionAssociationsT>
    class MotionVector3: public GeomVector3<MotionT, MotionAssociationsT>
    {
    public:
        /**
         * constructors
         */
        MotionVector3();
        MotionVector3(const double* in_data, const unsigned int in_size);
        MotionVector3(const MotionVector3 & other);
        virtual ~MotionVector3();
        
        /* Cross products */
        typename MotionAssociationsT::template Derivative<LinearMotionAssociationsT>::Type cross(const LinearMotionVector3& other) const;
        typename MotionAssociationsT::template Derivative<AngularMotionAssociationsT>::Type cross(const AngularMotionVector3& other) const;
        typename MotionAssociationsT::template Derivative<LinearForceAssociationsT>::Type cross(const LinearForceVector3& other) const;
        typename MotionAssociationsT::template Derivative<AngularForceAssociationsT>::Type cross(const AngularForceVector3& other) const;
    };
    
    /**
     * Method definitions
     */
    
    // constructors
    template <class MotionT, class MotionAssociationsT>
    MotionVector3<MotionT, MotionAssociationsT>::MotionVector3(): GeomVector3<MotionT, MotionAssociationsT>()
    {}
    
    template <class MotionT, class MotionAssociationsT>
    MotionVector3<MotionT, MotionAssociationsT>::MotionVector3(const double* in_data, const unsigned int in_size): GeomVector3<MotionT, MotionAssociationsT>(in_data, in_size)
    {}
    
    template <class MotionT, class MotionAssociationsT>
    MotionVector3<MotionT, MotionAssociationsT>::MotionVector3(const MotionVector3 & other): GeomVector3<MotionT, MotionAssociationsT>(other)
    {}
    
    template <class MotionT, class MotionAssociationsT>
    MotionVector3<MotionT, MotionAssociationsT>::~MotionVector3()
    {}

    /* Cross products */
    template <class MotionT, class MotionAssociationsT>
    typename MotionAssociationsT::template Derivative<LinearMotionAssociationsT>::Type MotionVector3<MotionT, MotionAssociationsT>::cross(const LinearMotionVector3& other) const
    {}
    
    template <class MotionT, class MotionAssociationsT>
    typename MotionAssociationsT::template Derivative<AngularMotionAssociationsT>::Type MotionVector3<MotionT, MotionAssociationsT>::cross(const AngularMotionVector3& other) const
    {}
    
    template <class MotionT, class MotionAssociationsT>
    typename MotionAssociationsT::template Derivative<LinearForceAssociationsT>::Type MotionVector3<MotionT, MotionAssociationsT>::cross(const LinearForceVector3& other) const
    {}
    
    template <class MotionT, class MotionAssociationsT>
    typename MotionAssociationsT::template Derivative<AngularForceAssociationsT>::Type MotionVector3<MotionT, MotionAssociationsT>::cross(const AngularForceVector3& other) const
    {}
}

#endif /* IDYNTREE_MOTION_VECTOR_3_H */