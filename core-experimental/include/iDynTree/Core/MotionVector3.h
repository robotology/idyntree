/*
 * Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
 * Authors: Nuno Guedelha
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef IDYNTREE_MOTION_VECTOR_3_H
#define IDYNTREE_MOTION_VECTOR_3_H

#include <iDynTree/Core/GeomVector3.h>

namespace iDynTree
{
    class LinearMotionVector3;
    class AngularMotionVector3;
    class LinearForceVector3;
    class AngularForceVector3;
    
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
    template <class MotionT, template <class MotionForceT> class MotionConversionsT>
    class MotionVector3: public GeomVector3<MotionT, MotionConversionsT>
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
        MotionT cross(const LinearMotionVector3& other) const;
        MotionT cross(const AngularMotionVector3& other) const;
        MotionT cross(const LinearForceVector3& other) const;
        MotionT cross(const AngularForceVector3& other) const;
        
        //typename MotionT::template derivative<AngularMotionVector3>.type cross(const AngularMotionVector3& other) const;
        //typename MotionT::derivative<LinearForceVector3>.type cross(const LinearForceVector3& other) const;
        //typename MotionT::derivative<AngularForceVector3>.type cross(const AngularForceVector3& other) const;
    };
    
    /**
     * Method definitions
     */
    
    // constructors
    template <class MotionT, template <class MotionForceT> class MotionConversionsT>
    MotionVector3<MotionT, MotionConversionsT>::MotionVector3(): GeomVector3<MotionT, MotionConversionsT>()
    {}
    
    template <class MotionT, template <class MotionForceT> class MotionConversionsT>
    MotionVector3<MotionT, MotionConversionsT>::MotionVector3(const double* in_data, const unsigned int in_size): GeomVector3<MotionT, MotionConversionsT>(in_data, in_size)
    {}
    
    template <class MotionT, template <class MotionForceT> class MotionConversionsT>
    MotionVector3<MotionT, MotionConversionsT>::MotionVector3(const MotionVector3 & other): GeomVector3<MotionT, MotionConversionsT>(other)
    {}
    
    template <class MotionT, template <class MotionForceT> class MotionConversionsT>
    MotionVector3<MotionT, MotionConversionsT>::~MotionVector3()
    {}

    /* Cross products */
    template <class MotionT, template <class MotionForceT> class MotionConversionsT>
    MotionT MotionVector3<MotionT, MotionConversionsT>::cross(const LinearMotionVector3& other) const
    {}
    
    template <class MotionT, template <class MotionForceT> class MotionConversionsT>
    MotionT MotionVector3<MotionT, MotionConversionsT>::cross(const AngularMotionVector3& other) const
    {}
    
    template <class MotionT, template <class MotionForceT> class MotionConversionsT>
    MotionT MotionVector3<MotionT, MotionConversionsT>::cross(const LinearForceVector3& other) const
    {}
    
    template <class MotionT, template <class MotionForceT> class MotionConversionsT>
    MotionT MotionVector3<MotionT, MotionConversionsT>::cross(const AngularForceVector3& other) const
    {}
}

#endif /* IDYNTREE_MOTION_VECTOR_3_H */